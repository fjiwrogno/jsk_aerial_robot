# State switch
## overview
Generally create two sets of parameters with same varible name to avoid too much edition on original pipeline
## basic parameter
use a node to publish the param according to currrent state
## state judger
can automatically judege the state or manually assign the mode for easy tuning.
after detecting the state, just publish the specific parameter without adding extra logic judgement loop
## specific function part
the original function will be used for running the main pipeline
