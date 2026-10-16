# Gui Architecture

```mermaid
classDiagram
    class MainWindow {
        +footer_info
        +alive
        +model_tab
        +telemetry_tab
        +btn_homing
        +ip_combo
        +robot
        +btn_connect
        +btn_pwr_mcu
        +btn_stop
        +label_status
        +configuration_tab
        +sim_backend
        +overwrite
        +btn_guide
        +control_tab
        +log
        +automatic_tab
        +__init__(robot_api)
        +show_message(message, timeout_ms)
        +_apply_show_message(message, timeout_ms)
        +_on_state_from_api(payload, info)
        +_on_overwrite_from_api(payload)
        +_start_sim_backend()
        +_stop_sim_backend()
        +_on_connect_clicked()
        +closeEvent(event)
    }
    class JogPage {
        +page_layout
        +fields
        +frame
        +__init__(frame, field_ids, field_tile, field_names, robot_api)
    }
    class JogPageHeader {
        +layout
        +__init__(title)
    }
    class JogPageField {
        +id
        +btn_jog_plus
        +layout
        +robot
        +name
        +btn_jog_minus
        +frame
        +__init__(field_id, field_name, frame, robot_api)
    }
    class PositionPage {
        +on_set_positions_changed
        +robot
        +commands
        +page_layout
        +fields
        +frame
        +__init__(frame, field_ids, field_tile, field_names, robot_api)
        +_on_set_changed()
        +set_from_actual()
    }
    class PositionPageHeader {
        +layout
        +__init__(title)
    }
    class PositionPageField {
        +id
        +label_act_pos
        +layout
        +robot
        +box_set_pos
        +name
        +log
        +frame
        +__init__(field_id, field_name, frame, robot_api)
        +set_from_actual()
        +_on_target_from_api(payload)
    }
    QObject <|-- PositionPageField
    class PositionPageCommands {
        +btn_move_linear
        +btn_set_from_file
        +layout
        +robot
        +btn_move_joint
        +ws
        +placeholder
        +btn_set_from_actual
        +fields
        +frame
        +__init__(fields, frame, robot_api)
        +read_from_file()
    }
    class ModelPageCommands {
        +btn_act_to_file
        +btn_set_to_file
        +layout
        +robot
        +ws
        +__init__(robot_api)
        +write_to_file(target)
    }
    class MCUConfigPage {
        +layout
        +fields
        +__init__(field_ids, robot_api)
    }
    class MCUConfigField {
        +layout
        +robot
        +field_id
        +btn_jump_to_bootloader
        +btn_set_zero
        +__init__(field_id, robot_api)
        +set_zero()
        +jump_to_bootloader()
    }
    class AutomaticTab {
        +btn_append_moveJ_guide
        +btn_refresh_programs
        +robot
        +btn_push_workspace
        +btn_clear_guide
        +btn_restart_once
        +btn_pull_workspace
        +ws
        +btn_restart_infinite
        +combo_programs
        +btn_cancel
        +btn_append_moveL_guide
        +log
        +btn_resume
        +btn_pause
        +__init__(robot_api)
        +resume()
        +pause()
        +cancel()
        +clear_guide()
        +append_moveJ()
        +append_moveL()
        +refresh_program_list()
        +_run_selected_program(repeat)
        +_on_pull_pressed()
        +_on_push_pressed()
        +_on_zmq_workspace_received(b64_payload)
    }
    class BaseConfigTableModel {
        +config
        +__init__(config)
        +update_config(config)
        +_on_config_updated()
        +_cast_value(old_val, new_val)
    }
    QAbstractTableModel <|-- BaseConfigTableModel
    class RobotConfigTableModel {
        +param_tooltips
        +param_names
        +__init__(config)
        +_on_config_updated()
        +rowCount(parent)
        +columnCount(parent)
        +data(index, role)
        +setData(index, value, role)
        +flags(index)
        +headerData(section, orientation, role)
    }
    BaseConfigTableModel <|-- RobotConfigTableModel
    class DataclassKeyValTableModel {
        +title_prefix
        +target_attr
        +keys
        +__init__(config, target_attr, title_prefix)
        +_on_config_updated()
        +rowCount(parent)
        +columnCount(parent)
        +data(index, role)
        +setData(index, value, role)
        +flags(index)
        +headerData(section, orientation, role)
    }
    BaseConfigTableModel <|-- DataclassKeyValTableModel
    class Matrix4x4TableModel {
        +target_attr
        +matrix_attr
        +__init__(config, target_attr, matrix_attr)
        +matrix()
        +rowCount(parent)
        +columnCount(parent)
        +data(index, role)
        +setData(index, value, role)
        +flags(index)
        +headerData(section, orientation, role)
    }
    BaseConfigTableModel <|-- Matrix4x4TableModel
    class ConfigurationTab {
        +tool_model
        +btn_read_from_mcu
        +model
        +table
        +mcu_config_page
        +robot
        +tcp_offset_model
        +btn_write_to_file
        +tool_table
        +tcp_offset_table
        +btn_write_to_mcu
        +ws
        +link_model
        +log
        +link_table
        +btn_read_from_file
        +__init__(robot_api)
        +_update_all_models()
        +read_from_file()
        +write_to_file()
        +read_from_mcu()
        +write_to_mcu()
        +_on_config_from_api(payload)
    }
    class ControlTab {
        +robot
        +relative_container
        +relative_mode
        +absolute_mode
        +joint_position_page_object
        +base_position_page_object
        +absolute_container
        +relative_stack
        +log
        +absolute_stack
        +joint_position_page
        +__init__(robot_api)
        +update_relative_ui(text)
        +update_absolute_ui(text)
    }
    class HomeTab {
        +__init__()
    }
    class ModelTab {
        +act_links
        +act_bboxes
        +act_tcp_frame
        +model_page_commands
        +robot
        +set_tcp_frame
        +view
        +base_frame
        +chk_show_bbox
        +ignored_pairs
        +set_links
        +log
        +__init__(robot_api)
        +_toggle_bbox_visibility(visible)
        +_on_target_from_api(payload)
        +update_set_positions(target)
        +_update_robot(transformations, gripper_val, links, tcp, bboxes)
        +_check_collisions(transforms)
        +_homogeneous_transform(item, transformation)
        +_create_frame(size)
        +_load_stl(path, color, bbox_list)
        +_load_links(color, bbox_list)
    }
    class TelemetryTab {
        +curves_1_set
        +plot_widget_1
        +btn_stop_logger
        +robot
        +_data_buffer
        +legend_2
        +btn_analyze
        +plot_widget_2
        +btn_start_logger
        +curves_1_act
        +legend_1
        +log
        +curves_2
        +_render_timer
        +__init__(robot_api)
        +_start_telemetry_logger()
        +_stop_telemetry_logger()
        +_analyze_telemetry()
        +_on_telemetry_from_api(payload)
        +_update_gui_plots()
    }
```