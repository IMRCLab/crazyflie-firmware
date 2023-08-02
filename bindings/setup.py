"""Compiles the cffirmware C extension."""

import distutils.command.build
from distutils.core import setup, Extension
import os

include = [
    "src/modules/interface",
    "src/hal/interface",
    "src/utils/interface/lighthouse",
    "src/utils/interface",
    "build/include/generated",
    "src/config",
    "src/drivers/interface",
    "src/lib/osqp/include",
]

fw_sources = [
    "src/modules/src/pptraj.c",
    "src/modules/src/pptraj_compressed.c",
    "src/modules/src/planner.c",
    "src/modules/src/collision_avoidance.c",
    "src/modules/src/controller_pid.c",
    "src/modules/src/position_controller_pid.c",
    "src/modules/src/attitude_pid_controller.c",
    "src/modules/src/pid.c",
    "src/utils/src/filter.c",
    "src/utils/src/num.c",
    "src/modules/src/controller_mellinger.c",
    "src/modules/src/power_distribution_quadrotor.c",
    "src/modules/src/controller_sjc.c",
    "src/modules/src/controller_lee.c",
    "src/modules/src/controller_lee_payload.c",
    "src/lib/osqp/src/osqp/auxil.c",
    "src/lib/osqp/src/osqp/error.c",
    "src/lib/osqp/src/osqp/kkt.c",
    "src/lib/osqp/src/osqp/lin_alg.c",
    "src/lib/osqp/src/osqp/osqp.c",
    "src/lib/osqp/src/osqp/proj.c",
    "src/lib/osqp/src/osqp/qdldl_interface.c",
    "src/lib/osqp/src/osqp/qdldl.c",
    "src/lib/osqp/src/osqp/scaling.c",
    "src/lib/osqp/src/osqp/util.c",
    "src/lib/osqp/src/osqp/workspace_2uav_2hp.c",
    "src/lib/osqp/src/osqp/workspace_3uav_2hp.c",
    "src/lib/osqp/src/osqp/workspace_4uav_5hp.c",
    "src/lib/osqp/src/osqp/workspace_5uav_9hp.c",
    "src/lib/osqp/src/osqp/workspace_6uav_14hp.c",
    "src/lib/osqp/src/osqp/workspace_7uav_42hp.c",
    "src/lib/osqp/src/osqp/workspace_8uav_56hp.c",
    "src/lib/osqp/src/osqp/workspace_9uav_72hp.c",
    "src/lib/osqp/src/osqp/workspace_10uav_90hp.c",
    "src/lib/osqp/src/osqp/workspace_3uav_2hp_rig.c",
    "src/lib/osqp/src/osqp/workspace_3uav_6hp_rig.c",
    "src/lib/osqp/src/osqp/workspace_4uav_12hp_rig.c",
    "src/lib/osqp/src/osqp/workspace_5uav_20hp_rig.c",  
    "src/lib/osqp/src/osqp/workspace_6uav_30hp_rig.c",
    "src/lib/osqp/src/osqp/workspace_7uav_42hp_rig.c",    
    "src/lib/osqp/src/osqp/workspace_8uav_56hp_rig.c",    
    "src/lib/osqp/src/osqp/workspace_9uav_72hp_rig.c",    
    "src/lib/osqp/src/osqp/workspace_10uav_90hp_rig.c",    
    "src/lib/osqp/src/osqp/workspace_2uav_1hp_rod.c",
    "src/lib/osqp/src/osqp/workspace_hyperplane.c",
    "src/lib/osqp/src/osqp/workspace_hyperplane_rb.c",
    "src/lib/osqp/src/osqp/workspace_compute_Fd_pair.c",
]

cffirmware = Extension(
    "_cffirmware",
    include_dirs=include,
    sources=fw_sources + ["build/cffirmware_wrap.c"],
    extra_compile_args=[
        "-O3",
        # The following flags are also used for compiling the actual firmware
        "-fno-strict-aliasing",
        "-Wno-address-of-packed-member",
    ],
)

# Override build command to specify custom "build" directory
class BuildCommand(distutils.command.build.build):
    def initialize_options(self):
        distutils.command.build.build.initialize_options(self)
        self.build_base = "build"

setup(
    name="cffirmware",
    version="1.0",
    cmdclass={"build": BuildCommand},
    ext_modules=[cffirmware]
)
