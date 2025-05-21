import yaml
import os
from collections import OrderedDict
import copy # Import the copy module

# Helper to represent OrderedDict in YAML output if needed for specific versions/cases
# For most modern PyYAML and Python 3.7+, dict order is preserved by default.
def represent_ordereddict(dumper, data):
    value = []
    for item_key, item_value in data.items():
        node_key = dumper.represent_data(item_key)
        node_value = dumper.represent_data(item_value)
        value.append((node_key, node_value))
    return yaml.nodes.MappingNode(u'tag:yaml.org,2002:map', value)

yaml.add_representer(OrderedDict, represent_ordereddict)

def deep_update(source, overrides):
    """
    Update a nested dictionary or similar mapping.
    Modify ``source`` in place.
    """
    for key, value in overrides.items():
        if isinstance(value, dict) and value:
            returned = deep_update(source.get(key, {}), value)
            source[key] = returned
        else:
            source[key] = overrides[key]
    return source

def merge_yaml_files(base_file_path, layer_specific_file_path, output_file_path):
    """
    Merges a layer-specific YAML configuration (e.g., for voxel_layer or stvl)
    into a base Nav2 parameters YAML file.

    The layer-specific file should contain a single top-level key which is the
    name of the layer (e.g., 'voxel_layer' or 'spatio_temporal_voxel_layer'),
    and its value is the configuration for that layer.

    This function will:
    1. Load the base and layer-specific YAML files.
    2. Determine the layer name from the layer-specific file.
    3. Insert the layer name into the 'plugins' list of 'local_costmap' and 'global_costmap'
       in the correct order.
    4. Add the layer's configuration block under 'local_costmap' and 'global_costmap'.
    5. Write the merged result to the output file.
    """
    try:
        with open(base_file_path, 'r') as f:
            base_params = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"Error: Base YAML file not found at {base_file_path}")
        return
    except yaml.YAMLError as e:
        print(f"Error parsing base YAML file {base_file_path}: {e}")
        return

    try:
        with open(layer_specific_file_path, 'r') as f:
            layer_config_content = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"Error: Layer-specific YAML file not found at {layer_specific_file_path}")
        return
    except yaml.YAMLError as e:
        print(f"Error parsing layer-specific YAML file {layer_specific_file_path}: {e}")
        return

    if not layer_config_content or not isinstance(layer_config_content, dict) or len(layer_config_content) != 1:
        print(f"Error: Layer-specific YAML file {layer_specific_file_path} should contain a single top-level key "
              "representing the layer name and its configuration.")
        return

    layer_name = list(layer_config_content.keys())[0]
    layer_params_block = layer_config_content[layer_name]

    # --- Update local_costmap ---
    if 'local_costmap' in base_params and 'local_costmap' in base_params['local_costmap'] and \
       'ros__parameters' in base_params['local_costmap']['local_costmap']:
        
        local_ros_params = base_params['local_costmap']['local_costmap']['ros__parameters']
        
        # Define the plugins list with correct order
        local_plugins = ["obstacle_layer", layer_name, "inflation_layer"]
        local_ros_params['plugins'] = local_plugins
        
        # Add the layer's configuration block
        # Use a deepcopy for the first assignment as well, to be safe,
        # or ensure layer_params_block is not modified elsewhere if it were to be reused.
        # For this specific logic, assigning it directly here is fine,
        # the crucial part is the copy for the global_costmap.
        local_ros_params[layer_name] = copy.deepcopy(layer_params_block) # MODIFIED: Added deepcopy here for safety, though the next one is more critical
        print(f"Updated local_costmap with layer: {layer_name}")
    else:
        print("Warning: 'local_costmap.local_costmap.ros__parameters' not found in base YAML. Skipping local_costmap update.")

    # --- Update global_costmap ---
    if 'global_costmap' in base_params and 'global_costmap' in base_params['global_costmap'] and \
       'ros__parameters' in base_params['global_costmap']['global_costmap']:

        global_ros_params = base_params['global_costmap']['global_costmap']['ros__parameters']

        # Define the plugins list with correct order
        global_plugins = ["static_layer", "obstacle_layer", layer_name, "inflation_layer"]
        global_ros_params['plugins'] = global_plugins

        # Add the layer's configuration block using a deepcopy
        # This ensures it's a distinct object from the one in local_costmap
        global_ros_params[layer_name] = copy.deepcopy(layer_params_block) # MODIFIED: Added deepcopy
        print(f"Updated global_costmap with layer: {layer_name}")
    else:
        print("Warning: 'global_costmap.global_costmap.ros__parameters' not found in base YAML. Skipping global_costmap update.")

    # --- Write the merged YAML to the output file ---
    try:
        with open(output_file_path, 'w') as f:
            # Disable aliases in the output
            yaml.dump(base_params, f, sort_keys=False, indent=2, Dumper=yaml.Dumper, allow_unicode=True, default_flow_style=False)
        print(f"Successfully merged YAML files. Output saved to: {output_file_path}")
    except Exception as e:
        print(f"Error writing merged YAML to {output_file_path}: {e}")


if __name__ == '__main__':
    # --- Configuration for the script ---
    # Get the directory of the current script
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Assuming config files are in a 'config' subdirectory relative to the script's package share
    # This needs to be robust if the script is not in the config dir itself.
    # For now, let's assume the script is run from a context where 'config' is accessible
    # or paths are constructed relative to the package.
    
    # Example: If your script is in 'nav2_jetson/scripts' and configs in 'nav2_jetson/config'
    pkg_config_dir = os.path.join(os.path.dirname(script_dir), 'config') # Go up one level from scripts, then into config

    # If your script is in 'config_test' and other configs are there too:
    # pkg_config_dir = script_dir 

    # Let's assume the script is in 'config_test' and other YAMLs are there too, as per previous context
    pkg_config_dir = script_dir 


    base_nav2_params_file = os.path.join(pkg_config_dir, "nav2_params_base.yaml") # Corrected path assumption

    # Choose which layer to insert: 'voxel' or 'stvl'
    layer_type_to_insert = 'voxel'
    # layer_type_to_insert = 'stvl' 

    if layer_type_to_insert == 'voxel':
        layer_specific_file = os.path.join(pkg_config_dir, "voxel_layer.yaml") # Corrected path assumption
        output_file = os.path.join(pkg_config_dir, "nav2_params_generated_with_voxel.yaml") # Corrected path assumption
    elif layer_type_to_insert == 'stvl':
        layer_specific_file = os.path.join(pkg_config_dir, "spatio_temporal_voxel_layer.yaml") # Corrected path assumption
        output_file = os.path.join(pkg_config_dir, "nav2_params_generated_with_stvl.yaml") # Corrected path assumption
    else:
        print(f"Error: Unknown layer_type_to_insert: {layer_type_to_insert}. Choose 'voxel' or 'stvl'.")
        exit()

    print(f"Base parameters file: {base_nav2_params_file}")
    print(f"Layer-specific file: {layer_specific_file}")
    print(f"Output file: {output_file}")

    # --- Run the merge function ---
    merge_yaml_files(base_nav2_params_file, layer_specific_file, output_file)

    # --- Example of how to verify (optional) ---
    # print("\n--- Verifying output (first few lines) ---")
    # try:
    #     with open(output_file, 'r') as f:
    #         for i in range(20): # Print first 20 lines
    #             line = f.readline()
    #             if not line:
    #                 break
    #             print(line, end='')
    # except FileNotFoundError:
    #     print(f"Could not read output file {output_file} for verification.")