#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import ParameterValueType
from rcl_interfaces.srv import GetParameters, SetParameters, ListParameters
import open3d.visualization.gui as gui
from functools import partial


class gPhoto2Monitor(Node):
    def __init__(self):
        super().__init__('gphoto2_monitor')

        self.target_node = '/camera1'  # Replace with actual target node name
        self.param_names = []
        self.param_values = {}

        self.get_parameter_names()

        gui.Application.instance.initialize()
        self.window = gui.Application.instance.create_window(
            'Parameter Adjuster', 400, 300)
        layout = gui.Vert(0, gui.Margins(8, 8, 8, 8))

        self.param_widgets = {}

        for param_name in self.param_names:
            param_value = gui.TextEdit(
            ) if self.param_values[param_name].type_ == ParameterValueType.PARAMETER_STRING else gui.IntEdit()
            param_value.set_on_value_changed(
                partial(self.on_value_changed, param_name))
            self.param_widgets[param_name] = param_value
            layout.add_child(gui.Label(param_name))
            layout.add_child(param_value)

        self.window.add_child(layout)

    def get_parameter_names(self):
        client = self.create_client(
            ListParameters, f'{self.target_node}/list_parameters')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for list parameters service...')

        request = ListParameters.Request()
        future = client.call_async(request)
        future.add_done_callback(self.handle_list_parameters)

    def handle_list_parameters(self, future):
        try:
            response = future.result()
            self.param_names = response.names
            self.get_logger().info(f'List of parameters: {self.param_names}')
            self.get_parameters()
        except Exception as e:
            self.get_logger().error(f'Failed to list parameters: {str(e)}')

    def get_parameters(self):
        client = self.create_client(
            GetParameters, f'{self.target_node}/get_parameters')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get parameters service...')

        request = GetParameters.Request()
        request.names = self.param_names
        future = client.call_async(request)
        future.add_done_callback(self.handle_get_parameters)

    def handle_get_parameters(self, future):
        try:
            response = future.result()
            for param_value in response.values:
                self.param_values[param_value.name] = param_value
                if param_value.type == ParameterValueType.PARAMETER_STRING:
                    self.param_widgets[param_value.name].text = param_value.string_value
                elif param_value.type == ParameterValueType.PARAMETER_INTEGER:
                    self.param_widgets[param_value.name].int_value = param_value.integer_value
        except Exception as e:
            self.get_logger().error(f'Failed to get parameters: {str(e)}')

    def set_parameter(self, param_name, value):
        client = self.create_client(
            SetParameters, f'{self.target_node}/set_parameters')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set parameters service...')

        from rclpy.parameter import Parameter, ParameterValue

        request = SetParameters.Request()
        param = Parameter()
        param.name = param_name
        param.value = ParameterValue()

        if isinstance(value, str):
            param.value.type = ParameterValueType.PARAMETER_STRING
            param.value.string_value = value
        elif isinstance(value, int):
            param.value.type = ParameterValueType.PARAMETER_INTEGER
            param.value.integer_value = value

        request.parameters.append(param)
        future = client.call_async(request)
        future.add_done_callback(
            partial(self.handle_set_parameter, param_name=param_name))

    def handle_set_parameter(self, future, param_name):
        try:
            response = future.result()
            self.get_logger().info(f'Parameter {param_name} set: {response}')
        except Exception as e:
            self.get_logger().error(
                f'Failed to set parameter {param_name}: {str(e)}')

    def on_value_changed(self, param_name, value):
        if isinstance(value, gui.TextEdit):
            self.set_parameter(param_name, value.text)
        elif isinstance(value, gui.IntEdit):
            self.set_parameter(param_name, value.int_value)


def main(args=None):
    rclpy.init(args=args)
    node = gPhoto2Monitor()
    try:
        gui.Application.instance.run()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
