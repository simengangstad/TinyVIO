#ifndef PIPELINE_H
#define PIPELINE_H

#include <stddef.h>
#include <stdint.h>

namespace vio {

    /**
     * @brief Runs the VIO pipeline with a proxy data link, streaming data over
     * ethernet.
     */
    void run_pipeline_with_proxy_link();
}

#endif
