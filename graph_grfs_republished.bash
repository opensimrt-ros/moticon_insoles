#!/usr/bin/env bash
set -e

rqt_plot \
	/grf_right/wrench/wrench/force/x:y:z \
	/grf_right/wrench/wrench/torque/x:y:z \
	/grf_left/wrench/wrench/force/x:y:z \
	/grf_left/wrench/wrench/torque/x:y:z \
	/grf_right/wrench_filtered/wrench/force/x:y:z \
	/grf_left/wrench_filtered/wrench/force/x:y:z \
	/grf_right/wrench_fixed_freq/wrench/force/x:y:z \
	/grf_left/wrench_fixed_freq/wrench/force/x:y:z

