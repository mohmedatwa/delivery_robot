#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ClearEntireCostmap
import threading
import tkinter as tk

class ClearCostmapNode(Node):
    def __init__(self):
        super().__init__('clear_costmap_gui')
        self.global_cli = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap')
        self.local_cli = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap')

    def clear(self, client, label_var, name):
        if not client.service_is_ready():
            label_var.set(f'{name}: service not ready')
            return
        label_var.set(f'Clearing {name}...')
        req = ClearEntireCostmap.Request()
        future = client.call_async(req)
        future.add_done_callback(
            lambda f: label_var.set(f'✓ {name} cleared'))

def main():
    rclpy.init()
    node = ClearCostmapNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    root = tk.Tk()
    root.title('Nav2 Costmap Tools')
    root.geometry('280x120')
    status = tk.StringVar(value='Ready')

    tk.Button(root, text='Clear Global Costmap', bg='#c0392b', fg='white',
        font=('Helvetica', 11, 'bold'),
        command=lambda: node.clear(node.global_cli, status, 'Global')
    ).pack(fill='x', padx=10, pady=(10,2))

    tk.Button(root, text='Clear Local Costmap', bg='#e67e22', fg='white',
        font=('Helvetica', 11, 'bold'),
        command=lambda: node.clear(node.local_cli, status, 'Local')
    ).pack(fill='x', padx=10, pady=2)

    tk.Label(root, textvariable=status).pack(pady=4)
    root.mainloop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
