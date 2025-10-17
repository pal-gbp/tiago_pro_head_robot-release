# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from tiago_pro_head_description.launch_arguments import TiagoProHeadArgs

from urdf_test.xacro_test import define_xacro_test

xacro_file_path = Path(
    get_package_share_directory('tiago_pro_head_description'),
    'robots',
    'tiago_pro_head.urdf.xacro',
)


test_xacro_base = define_xacro_test(
    xacro_file_path, TiagoProHeadArgs.camera_model)
