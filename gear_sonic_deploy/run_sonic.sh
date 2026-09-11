#!/usr/bin/env bash
# SONIC deploy, same flags as teleop and tools/vla-inference.sh (run INSIDE the g1-deploy-dev container).
cd /workspace/g1_deploy
exec ./target/release/g1_deploy_onnx_ref enP2p1s0 policy/sonic_v1_1/model_decoder.onnx reference/example/ --obs-config policy/sonic_v1_1/observation_config.yaml --encoder-file policy/sonic_v1_1/model_encoder.onnx --planner-file planner/target_vel/V2/planner_sonic.onnx --input-type zmq_manager --output-type all --zmq-host localhost
