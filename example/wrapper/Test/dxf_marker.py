import time
import ezdxf
import math
from xarm.wrapper import XArmAPI

# Connexion au robot
arm = XArmAPI('192.168.1.215', baud_checkset=False)

# Initialisation du robot
arm.clean_warn()
arm.clean_error()
arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)
time.sleep(1)

# Paramètres du robot
TCP_SPEED = 800
TCP_ACC = 20000
Z_HEIGHT = 50  # Hauteur de sécurité
DRAW_HEIGHT = 5  # Hauteur pour dessiner

# Charger le fichier DXF
dxf_file = "ki.dxf"  # Remplace par ton fichier
doc = ezdxf.readfile(dxf_file)
msp = doc.modelspace()

# Facteur d'échelle pour ajuster la taille du dessin
scale_factor = 1.0

# Déplacer en hauteur de sécurité avant de commencer
arm.set_position(300, 0, Z_HEIGHT, 180.0, 0.0, 0.0, speed=TCP_SPEED, mvacc=TCP_ACC, wait=True)

# Lire les entités du DXF
for entity in msp:
    if entity.dxftype() == "LINE":
        print("Dessin d'une ligne...")
        x1, y1 = (entity.dxf.start.x * scale_factor, entity.dxf.start.y * scale_factor)
        x2, y2 = (entity.dxf.end.x * scale_factor, entity.dxf.end.y * scale_factor)

        #offset of 300 for x and 100 for y
        x1 += 300
        y1 += 100
        x2 += 300
        y2 += 100


        # Mouvement pour tracer la ligne
        arm.set_position(x2, y2, DRAW_HEIGHT, 180.0, 0.0, 0.0, speed=TCP_SPEED, mvacc=TCP_ACC, radius=0, wait=True)



    elif entity.dxftype() == "ARC":
        print("Dessin d'un arc...")
        cx, cy = entity.dxf.center.x * scale_factor, entity.dxf.center.y * scale_factor
        radius = entity.dxf.radius * scale_factor
        start_angle = math.radians(entity.dxf.start_angle)
        end_angle = math.radians(entity.dxf.end_angle)

        #offset of 300 for x and 100 for y
        cx += 300
        cy += 100

        # Diviser l'arc en petits segments
        num_points = 20
        for i in range(num_points + 1):
            angle = start_angle + (end_angle - start_angle) * (i / num_points)
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)

            if i == 0:
                arm.set_position(x, y, Z_HEIGHT, 180.0, 0.0, 0.0, speed=TCP_SPEED, mvacc=TCP_ACC, wait=True)
                arm.set_position(x, y, DRAW_HEIGHT, 180.0, 0.0, 0.0, speed=TCP_SPEED, mvacc=TCP_ACC, wait=True)
            else:
                arm.set_position(x, y, DRAW_HEIGHT, 180.0, 0.0, 0.0, speed=TCP_SPEED, mvacc=TCP_ACC, wait=True)


    elif entity.dxftype() == "CIRCLE":
        cx, cy = entity.dxf.center.x * scale_factor, entity.dxf.center.y * scale_factor
        radius = entity.dxf.radius * scale_factor

        num_points = 36  # Diviser le cercle en 36 points
        points = []

        for i in range(num_points + 1):
            angle = 2 * math.pi * (i / num_points)  # Angle en radians
            x = cx + radius * math.cos(angle)+300
            y = cy + radius * math.sin(angle)+100
            points.append((x, y))

        # Déplacer en hauteur de sécurité avant de commencer
        arm.set_position(points[0][0], points[0][1], Z_HEIGHT, 180.0, 0.0, 0.0, speed=TCP_SPEED, mvacc=TCP_ACC, wait=True)
        arm.set_position(points[0][0], points[0][1], DRAW_HEIGHT, 180.0, 0.0, 0.0, speed=TCP_SPEED, mvacc=TCP_ACC, wait=True)

        # Tracer le cercle en suivant les points
        for x, y in points:
            arm.set_position(x, y, DRAW_HEIGHT, 180.0, 0.0, 0.0, speed=TCP_SPEED, mvacc=TCP_ACC, radius=0, wait=False)

        # Remonter après avoir dessiné le cercle
        arm.set_position(points[-1][0], points[-1][1], Z_HEIGHT, 180.0, 0.0, 0.0, speed=TCP_SPEED, mvacc=TCP_ACC, wait=True)
 
    else:
        print("Entité inconnue:", entity.dxftype())

 

# Fin du programme
arm.disconnect()
print("Dessin terminé !")
