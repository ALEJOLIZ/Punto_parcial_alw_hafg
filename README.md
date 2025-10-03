Notas:
En el traj_plan_node.py : Se tiene que cambiar las rutas absolutas por las que se tengan al clonar el repositorio; en las lineas 19 (        with open('/home/alejo/ros2_trayectoria/src/trayectoria_parcial/dxf/dxf_waypoints_v5.csv', 'r') as file: ) y en la 197 ( output_path = "/home/alejo/ros2_trayectoria/src/trayectoria_parcial/dxf/trajectory.csv" )
En el dxf_exporter_node.py :  Se tiene que cambiar las rutas absolutas por las que se tengan al clonar el repositorio; en las lineas 34 (        self.declare_parameter('dxf_file', '/home/alejo/ros2_trayectoria/src/trayectoria_parcial/dxf/Test_2.dxf') ) y en la 169 (    def export_to_csv(self, filename='/home/alejo/ros2_trayectoria/src/trayectoria_parcial/dxf/dxf_waypoints_v5.csv'): )
Para probar la planeacion de trayectoria:
1. Se tiene que escribir en el wsl la siguiente linea: ros2 launch trayectoria_parcial robot_launch_alw_hafg.launch.py
2. Se espera a que se abra el rviz con los diferentes TF y robot_model y ahi se ejecuta la siguiente linea: ros2 run trayectoria_parcial traj_plan_node 
Con estos pasos y al seguir las notas se puede vizualizar la planeacion de trayectoria del .dxf que se hizo.
