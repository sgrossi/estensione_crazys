Il lavoro è un'espansione del già esistente progetto CrazyS di G.Silano per scopo di tesi.
https://github.com/gsilano/CrazyS

La presente repository è strutturata come segue:

- Le cartelle "lee_controller", "standard_controller" e "trajectory_nodes" vanno inserite nella cartella src di un workspace catkin. Successivamente la loro compilazione si fa con catkin_make come da istruzioni nei tutorial di ROS

- La cartella "matlab_lee_controller" contiene script per MatLab e Simulink che implementano il controllore di Lee e ne permettono l'utilizzo facendo in modo che il software comunichi con ROS grazie a Robotics System Toolbox

- Il contenuto della cartella "launch" può essere copiato all'interno dell'omonima nel package rotors_gazebo di CrazyS. Questi file sfruttano i due controllori inclusi in questa repository.

Per maggiori informazioni si rimanda alla tesi di riferimento.
