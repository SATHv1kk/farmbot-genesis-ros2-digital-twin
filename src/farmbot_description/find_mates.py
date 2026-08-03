# Copyright 2026 Sathvik
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

"""Find and list mate features from an Onshape assembly document."""

import json
import os
import sys

from onshape_to_robot.onshape_api.client import Client

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(SCRIPT_DIR, 'config.json')

try:
    with open(CONFIG_PATH, 'r') as f:
        config = json.load(f)
except FileNotFoundError:
    print(f'ERROR: config.json not found at {CONFIG_PATH}')
    sys.exit(1)

doc_id = config.get('documentId')
ws_id = config.get('workspaceId')
el_id = config.get('elementId')

if not all([doc_id, ws_id, el_id]):
    print(
        'ERROR: documentId, workspaceId, or elementId is missing '
        'from config.json'
    )
    sys.exit(1)

try:
    client = Client(logging=False)
    print(
        f'Connecting to Onshape and fetching assembly data '
        f'for element {el_id}...'
    )

    assembly_data = client.get_assembly(doc_id, ws_id, el_id)

    if 'mateFeatures' in assembly_data:
        print('\n--- Found Mate Features ---')
        for mate_feature in assembly_data['mateFeatures']:
            if 'name' in mate_feature:
                print(f"- {mate_feature['name']}")
        print('---------------------------\n')
        print(
            'SUCCESS: Please look through the list above for the '
            'names of your SLIDER mates.'
        )
    else:
        print(
            "\nERROR: Could not find 'mateFeatures' in the data "
            'returned by Onshape.'
        )

except Exception as e:  # noqa: B902 -- CLI helper, catch-all intentional
    print(f'\nAn error occurred: {e}')
    sys.exit(1)
