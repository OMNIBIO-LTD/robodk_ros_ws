#!/usr/bin/env python3
"""
Small pygame UI that publishes gripper open/close commands.

Publishes std_msgs/Float64 on /gripper_command:
    -25.0  -> open
      0.0  -> stop
     25.0  -> close

The joint_state_streamer subscribes to this topic and appends the value
to the published JointState as the gripper_slider joint position.
"""

import sys

import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


OPEN_VALUE = -25.0
STOP_VALUE = 0.0
CLOSE_VALUE = 25.0


class GripperButton(Node):
    def __init__(self):
        super().__init__('gripper_button')
        self.declare_parameter('topic', '/gripper_command')
        self.declare_parameter('initial_state', OPEN_VALUE)
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.state = float(
            self.get_parameter('initial_state').get_parameter_value().double_value
        )
        self.pub = self.create_publisher(Float64, topic, 10)
        self.get_logger().info(f'Publishing gripper commands on "{topic}"')

    def publish_state(self):
        msg = Float64()
        msg.data = float(self.state)
        self.pub.publish(msg)
        if self.state == OPEN_VALUE:
            label = 'OPEN'
        elif self.state == CLOSE_VALUE:
            label = 'CLOSE'
        elif self.state == STOP_VALUE:
            label = 'STOP'
        else:
            label = f'{self.state:.1f}'
        self.get_logger().info(f'gripper -> {label} ({self.state:.1f})')


def _draw_button(surface, rect, label, base_color, font, hovered, pressed):
    color = base_color
    if pressed:
        color = tuple(max(0, c - 40) for c in base_color)
    elif hovered:
        color = tuple(min(255, c + 20) for c in base_color)
    pygame.draw.rect(surface, color, rect, border_radius=10)
    pygame.draw.rect(surface, (30, 30, 30), rect, 2, border_radius=10)
    text = font.render(label, True, (20, 20, 20))
    surface.blit(text, text.get_rect(center=rect.center))


def main(args=None):
    rclpy.init(args=args)
    node = GripperButton()

    pygame.init()
    width, height = 480, 240
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption('Gripper Control')
    button_font = pygame.font.SysFont(None, 36)
    title_font = pygame.font.SysFont(None, 34)
    status_font = pygame.font.SysFont(None, 26)

    open_rect = pygame.Rect(30, 130, 120, 80)
    stop_rect = pygame.Rect(180, 130, 120, 80)
    close_rect = pygame.Rect(330, 130, 120, 80)

    OPEN_COLOR = (140, 220, 140)
    STOP_COLOR = (235, 215, 130)
    CLOSE_COLOR = (235, 150, 150)

    clock = pygame.time.Clock()
    running = True
    pressed_rect = None

    node.publish_state()

    try:
        while running and rclpy.ok():
            mouse_pos = pygame.mouse.get_pos()
            mouse_pressed = pygame.mouse.get_pressed()[0]

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_ESCAPE, pygame.K_q):
                        running = False
                    elif event.key == pygame.K_o:
                        node.state = OPEN_VALUE
                        node.publish_state()
                    elif event.key in (pygame.K_c, pygame.K_SPACE):
                        node.state = CLOSE_VALUE
                        node.publish_state()
                    elif event.key == pygame.K_s:
                        node.state = STOP_VALUE
                        node.publish_state()
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    if open_rect.collidepoint(event.pos):
                        pressed_rect = open_rect
                    elif stop_rect.collidepoint(event.pos):
                        pressed_rect = stop_rect
                    elif close_rect.collidepoint(event.pos):
                        pressed_rect = close_rect
                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    if pressed_rect is open_rect and open_rect.collidepoint(event.pos):
                        node.state = OPEN_VALUE
                        node.publish_state()
                    elif pressed_rect is stop_rect and stop_rect.collidepoint(event.pos):
                        node.state = STOP_VALUE
                        node.publish_state()
                    elif pressed_rect is close_rect and close_rect.collidepoint(event.pos):
                        node.state = CLOSE_VALUE
                        node.publish_state()
                    pressed_rect = None

            screen.fill((28, 28, 32))
            title = title_font.render('Gripper Control', True, (240, 240, 240))
            screen.blit(title, title.get_rect(center=(width // 2, 32)))

            if node.state == OPEN_VALUE:
                state_label = 'OPEN'
                state_color = (160, 230, 160)
            elif node.state == CLOSE_VALUE:
                state_label = 'CLOSED'
                state_color = (235, 160, 160)
            elif node.state == STOP_VALUE:
                state_label = 'STOP'
                state_color = (240, 220, 140)
            else:
                state_label = f'{node.state:.1f}'
                state_color = (220, 220, 220)

            status = status_font.render(
                f'state: {state_label}  (publishing {node.state:.1f})',
                True, state_color
            )
            screen.blit(status, status.get_rect(center=(width // 2, 75)))

            hint = status_font.render(
                "keys: O = open   S = stop   C/space = close   Q/Esc = quit",
                True, (150, 150, 150)
            )
            screen.blit(hint, hint.get_rect(center=(width // 2, 105)))

            _draw_button(
                screen, open_rect, 'OPEN', OPEN_COLOR, button_font,
                hovered=open_rect.collidepoint(mouse_pos),
                pressed=pressed_rect is open_rect and mouse_pressed,
            )
            _draw_button(
                screen, stop_rect, 'STOP', STOP_COLOR, button_font,
                hovered=stop_rect.collidepoint(mouse_pos),
                pressed=pressed_rect is stop_rect and mouse_pressed,
            )
            _draw_button(
                screen, close_rect, 'CLOSE', CLOSE_COLOR, button_font,
                hovered=close_rect.collidepoint(mouse_pos),
                pressed=pressed_rect is close_rect and mouse_pressed,
            )

            pygame.display.flip()
            rclpy.spin_once(node, timeout_sec=0.0)
            clock.tick(30)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
