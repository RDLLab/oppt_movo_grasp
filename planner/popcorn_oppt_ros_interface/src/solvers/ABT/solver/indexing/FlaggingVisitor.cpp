/** @file FlaggingVisitor.cpp
 *
 * Contains the implementation of the FlaggingVisitor class.
 */
#include "FlaggingVisitor.hpp"

#include "ABT/solver/changes/ChangeFlags.hpp"
#include "ABT/solver/StatePool.hpp"

namespace abt {

FlaggingVisitor::FlaggingVisitor(StatePool *pool,
        ChangeFlags flagsToSet) :
                SpatialIndexVisitor(pool),
                flagsToSet_(flagsToSet) {
}

void FlaggingVisitor::visit(StateInfo* info) {
    getStatePool()->setChangeFlags(info, flagsToSet_);
}

} /* namespace abt */
