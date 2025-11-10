# Bolide Obstacle Avoidance

Ce package ROS2 implémente un algorithme d'évitement d'obstacles pour la voiture autonome Bolide.

## Description

L'algorithme analyse les données du Lidar pour détecter les obstacles dans trois secteurs :
- Gauche (90° à 150°)
- Devant (150° à 210°)
- Droite (210° à 270°)

Si un obstacle est détecté devant (distance moyenne < seuil), l'algorithme choisit le côté avec le plus d'espace libre pour tourner.

## Paramètres

- `front_threshold` : Distance seuil pour détecter un obstacle devant (défaut : 0.5m)
- `side_threshold` : Distance seuil pour les côtés (défaut : 0.3m)
- `turn_speed` : Vitesse de rotation normalisée (défaut : 0.05)
- `forward_speed` : Vitesse en avant (défaut : 0.1 m/s)
- `debug` : Activer les logs de debug (défaut : True)

## Topics

### Subscribers
- `/scan` (sensor_msgs/LaserScan) : Données du Lidar
- `/emergency_stop` (std_msgs/Bool) : Signal d'arrêt d'urgence

### Publishers
- `/cmd_vel` (std_msgs/Float32) : Vitesse de la voiture
- `/cmd_dir` (std_msgs/Float32) : Direction de la voiture

## Lancement

```bash
ros2 run bolide_obstacle_avoidance obstacle_avoidance_node
```

## Architecture

Ce package fait partie du niveau haut (high_level) et s'appuie sur les packages low_level pour :
- Recevoir les données Lidar
- Publier les commandes de vitesse et direction
- Intégrer avec le système d'arrêt d'urgence