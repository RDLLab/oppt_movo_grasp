#ifndef _MULTITHREADED_ABT_OPTIONS_HPP_
#define _MULTITHREADED_ABT_OPTIONS_HPP_
#include "ABT/ABTOptions.hpp"

using namespace oppt;

namespace solvers
{
struct MultithreadedABTOptions: public ABTExtendedOptions {
public:
    MultithreadedABTOptions() = default;
    virtual ~MultithreadedABTOptions() = default;

    unsigned long maxNumHistories = 1;    

    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
        std::unique_ptr<options::OptionParser> parser =
            ABTExtendedOptions::makeParser(simulating);
        addMultithreadedABTOptions(parser.get());
        return std::move(parser);
    }

    static void addMultithreadedABTOptions(options::OptionParser* parser) {
        parser->addOption<unsigned long>("MultithreadedABT", "maxNumHistories",
                                         &MultithreadedABTOptions::maxNumHistories);        
    }

};
}

#endif
