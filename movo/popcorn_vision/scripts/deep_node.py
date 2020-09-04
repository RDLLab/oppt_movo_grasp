#!/usr/bin/env python

import sys
import os
import json
import numpy as np
import rospy
import ros_numpy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Int32
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from popcorn_vision.srv import GetMarkers, GetMarkersResponse
import time

import tensorflow as tf

file_dir = os.path.dirname(os.path.realpath(__file__))
config_path = os.path.join(file_dir, 'deep_config.json')
tf_research_dir = '/home/movo/tensorflow/models/research'
sys.path.append(tf_research_dir)

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

labels_dir = os.path.join(tf_research_dir, 'object_detection', 'data')
graphs_dir = os.path.join(tf_research_dir, 'object_detection', 'node_models')

min_score_thresh = 0.2

# kinect2 qhd is 960x540
# Crop image to zoom in
width = 600
height = 400

# Don't use very large detections - temporary fix for tube
max_detection_area = 70000

saved_count = 0

enabled = True

bridge = CvBridge()
image_sub = None
image_pub = None
marker_pub = None

detection_graph = None
sess = None
category_index = None

last_time = None
execution_time = None

img_cache = []
max_img_cache_size = 5
last_cloud = None

def detect_objects(image_np):
    global detection_graph, sess, sess2, category_index

    # Crop image
    row_off = (image_np.shape[0]-height)/2
    col_off = (image_np.shape[1]-width)/2
    image_np = image_np[row_off:row_off+height, col_off:col_off+width]

    #image_np = (image_np * 0.7).astype(np.uint8)

    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
    image_np_expanded = np.expand_dims(image_np, axis=0)
    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

    # Each box represents a part of the image where a particular object was detected.
    boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

    # Each score represent how level of confidence for each of the objects.
    # Score is shown on the result image, together with the class label.
    scores = detection_graph.get_tensor_by_name('detection_scores:0')
    classes = detection_graph.get_tensor_by_name('detection_classes:0')
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')

    # Actual detection.
    (boxes, scores, classes, num_detections) = sess.run(
            [boxes, scores, classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})
    boxes = np.squeeze(boxes)
    classes = np.squeeze(classes).astype(np.int32)
    scores = np.squeeze(scores)

    # Discard low confidence detections. Assumes that scores are ordered highest to lowest
    keep = []
    for i in range(len(scores)):
        if scores[i] < min_score_thresh:
            break
        # If this overlaps with previous detection, don't keep
        ymin, xmin, ymax, xmax = boxes[i]
        y = (ymax + ymin)/2
        x = (xmax + xmin)/2
        keep_this = True
        for j in keep:
            ymin2, xmin2, ymax2, xmax2 = boxes[j]
            if x > xmin2 and x < xmax2 and y > ymin2 and y < ymax2:
                keep_this = False
                break
        if keep_this:
            keep.append(i)
    boxes = np.array([boxes[i] for i in keep])
    classes = np.array([classes[i] for i in keep])
    scores = np.array([scores[i] for i in keep])

    # Visualization of the results of a detection.
    vis_util.visualize_boxes_and_labels_on_image_array(
        image_np,
        boxes,
        classes,
        scores,
        category_index,
        use_normalized_coordinates=True,
        min_score_thresh=min_score_thresh,
        line_thickness=5)

    return image_np, boxes, scores, classes

def process_image_msg(img):
    try:
        cv_img = bridge.imgmsg_to_cv2(img, "bgr8")
    except CvBridgeError as e:
        print(e)

    output_img, boxes, scores, classes = detect_objects(cv_img)

    detections = []
    for i in range(len(scores)):
        name = category_index[classes[i]]['name']
        detections.append({'name': name, 'score': scores[i], 'box': boxes[i]})

    cv2.imshow("Image window", output_img)
    cv2.waitKey(30)

    return detections

def color_callback(data):
    """Run CNNs when color image is received."""
    global last_time, execution_time, img_cache

    img_cache.append(data)
    if len(img_cache) > max_img_cache_size:
        img_cache.pop(0)

    if not enabled:
        return
    print 'Got msg'
    if last_time is None:
        last_time = time.time()

    detections = process_image_msg(data)

    took = time.time() - last_time
    last_time = time.time()
    if execution_time is None:
        execution_time = took
    else:
        execution_time = 0.2 * took + 0.8 * execution_time
    print 'Avg execution time: {} seconds'.format(execution_time)

def cloud_callback(data):
    global last_cloud
    last_cloud = data

def get_markers_cb(req):
    start_time = rospy.Time.now()
    while last_cloud is None:
        rospy.loginfo('Waiting for cloud')
        rospy.sleep(1.0)

    # Get the rgb image with timestamp closest to cloud
    min_index = -1
    min_diff = 999999
    for i in range(len(img_cache)):
        diff = img_cache[i].header.stamp - last_cloud.header.stamp
        diff = abs(diff.to_sec())
        if diff < min_diff:
            min_diff = diff
            min_index = i
    if min_diff > 1.0:
        rospy.logwarn('get_markers_cb: Large time mismatch. {} imgs in cache, closest is {}'.format(len(img_cache), min_diff))

    detections = process_image_msg(img_cache[min_index])

    cloud = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(last_cloud, remove_nans=False)

    markers = MarkerArray()

    for d in detections:
        ymin, xmin, ymax, xmax = d['box']

        # Un-normalize from float (between 0 to 1) to pixel coordinates
        # Also set offset to fix coordinates due to previous crop
        yoff = (cloud.shape[0] - height) / 2
        xoff = (cloud.shape[1] - width) / 2
        xmax = xoff + int(width * xmax)
        xmin = xoff + int(width * xmin)
        ymax = yoff + int(height * ymax)
        ymin = yoff + int(height * ymin)

        # Check detection is not too big
        area = (xmax - xmin) * (ymax - ymin)
        if area > max_detection_area:
            continue

        # Get small box around center
        cx = int((xmax + xmin) / 2.0)
        cy = int((ymax + ymin) / 2.0)
        box_size = 10
        bxmin = max(cx - box_size, 0)
        bxmax = min(cx + box_size, cloud.shape[1])
        bymin = max(cy - box_size, 0)
        bymax = min(cy + box_size, cloud.shape[0])

        # Get average real world coordinates
        num_real = 0
        avgxyz = np.zeros(3)

        for x in range(bxmin, bxmax):
            for y in range(bymin, bymax):
                xyz = cloud[y][x]
                if any([np.isnan(i) for i in xyz]):
                    continue
                avgxyz += xyz
                num_real += 1
        if num_real == 0:
            continue
        avgxyz /= num_real
        print 'Got {} non-nan pixels, avg xyz: {}'.format(num_real, avgxyz)

        marker = Marker()
        marker.header.frame_id = last_cloud.header.frame_id
        marker.header.stamp = rospy.Time.now()
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        marker.pose = pose
        marker.ns = 'deep_detections'
        marker.id = len(markers.markers)
        marker.text = d['name']
        marker.type = Marker.TEXT_VIEW_FACING
        marker.scale.z = 0.1
        marker.color.a = 1
        marker.color.g = 1
        markers.markers.append(marker)

    if len(markers.markers) > 0:
        marker_pub.publish(markers)

    d = rospy.Time.now() - start_time
    rospy.loginfo("Get pose took {} seconds".format(d.to_sec()))

    resp = GetMarkersResponse()
    resp.markers = markers
    return resp

def init_deep(deep_model):
    """Load CNN model. deep_model is config name, e.g. 'coco', 'robo_v0', 'tube'."""
    global detection_graph, sess, category_index

    model_details = json.load(open(config_path))[deep_model]

    print 'Init deep'

    graph_path = os.path.join(graphs_dir, model_details['graph'])
    labels_path = os.path.join(labels_dir, model_details['labels'])

    category_index = label_map_util.create_category_index_from_labelmap(labels_path)

    print 'Loading tf model into memory'
    config = tf.ConfigProto()
    detection_graph = tf.Graph()

    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(graph_path, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

        sess = tf.Session(graph=detection_graph, config=config)

    print 'Done'

def enable_cb(req):
    global enabled
    enabled = req.data
    resp = SetBoolResponse()
    resp.success = True
    return resp

def main():
    global image_sub, image_pub, pose_pub, marker_pub

    rospy.init_node('deep_node')
    deep_model = rospy.get_param('~deep_model', 'tube')
    rospy.loginfo("Deep model: {}".format(deep_model))
    init_deep(deep_model)

    image_sub = rospy.Subscriber("/movo_camera/qhd/image_color", Image, color_callback, queue_size=1, buff_size=2**24)
    cloud_sub = rospy.Subscriber("/movo_camera/point_cloud/points", PointCloud2, cloud_callback, queue_size=1)

    #int_pub = rospy.Publisher("/tensorflow_msg", Int32, queue_size=1)
    #image_pub = rospy.Publisher("image_topic_2", Image, queue_size=1)
    marker_pub = rospy.Publisher("deep_detections", MarkerArray, queue_size=1)

    enable_srv = rospy.Service('/enable_deep_node', SetBool, enable_cb)
    get_markers_srv = rospy.Service('/get_deep_detections', GetMarkers, get_markers_cb)

    rospy.spin()

if __name__ == '__main__':
    main()
