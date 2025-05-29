from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import RegisterEventHandler, EmitEvent, LogInfo
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    node_executable_name = 'detect' # The name of your python script
    lifecycle_node_name = 'fire_detect_node' # The name you give to your node in its constructor

    fire_detect_lifecycle_node_action = LifecycleNode(
        package='fire_bussiness',
        executable=node_executable_name,
        name=lifecycle_node_name,
        namespace='',
        parameters=[],
        autostart=True,
        # respawn=True,
        # respawn_delay=2.0,
    )

    # Event to trigger initial configuration
    trigger_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda action: action.node_name == lifecycle_node_name,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # Event handler to trigger activation after successful configuration
    trigger_activate_after_configure_success_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=fire_detect_lifecycle_node_action, # Pass the LifecycleNode action object
            start_state='configuring',
            goal_state='inactive', # 'inactive' means configuration was successful
            entities=[
                LogInfo(msg=f"Node '{lifecycle_node_name}' configured successfully. Triggering activation..."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda action: action.node_name == lifecycle_node_name,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    # Optional: Event handler to log successful activation
    log_successful_activation_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=fire_detect_lifecycle_node_action, # Pass the LifecycleNode action object
            start_state='activating',
            goal_state='active',
            entities=[
                LogInfo(msg=f"Node '{lifecycle_node_name}' activated successfully and is now active.")
            ],
        )
    )
    
    # Optional: Event handler to log configuration failure
    log_configuration_failure_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=fire_detect_lifecycle_node_action, # Pass the LifecycleNode action object
            start_state='configuring',
            goal_state='unconfigured', # 'unconfigured' means on_configure returned FAILURE
            entities=[
                LogInfo(msg=f"Node '{lifecycle_node_name}' failed to configure (transitioned to unconfigured). If node exits, respawn will occur.")
            ],
        )
    )

    return LaunchDescription([
        fire_detect_lifecycle_node_action,
        trigger_configure_event,
        trigger_activate_after_configure_success_handler,
        log_successful_activation_handler,
        log_configuration_failure_handler,
    ])