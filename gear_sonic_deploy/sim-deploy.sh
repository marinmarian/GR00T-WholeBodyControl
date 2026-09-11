#!/bin/bash
# SONIC deploy against the MuJoCo sim (run_sim_loop.py on the host).
#
# Host:              ./sim-deploy.sh            -> enters the dev container
# Inside container:  ./sim-deploy.sh            -> VR mode (streamer/headset starts control, like the real robot)
#                    ./sim-deploy.sh keyboard   -> keyboard mode (']' starts control, T/N/P play motions)
cd "$(dirname "$0")"
if [ ! -f /.dockerenv ]; then
    echo "Entering the dev container — once inside, run:  ./sim-deploy.sh  (or ./sim-deploy.sh keyboard)"
    exec ./docker/run-ros2-dev.sh
fi
INPUT_TYPE="${1:-zmq_manager}"
exec ./target/release/g1_deploy_onnx_ref lo \
    policy/sonic_v1_1/model_decoder.onnx reference/example/ \
    --obs-config policy/sonic_v1_1/observation_config.yaml \
    --encoder-file policy/sonic_v1_1/model_encoder.onnx \
    --planner-file planner/target_vel/V2/planner_sonic.onnx \
    --input-type "$INPUT_TYPE" --output-type all --zmq-host localhost \
    --disable-crc-check
