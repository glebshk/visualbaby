import xml.etree.ElementTree as ET
import random

tree = ET.parse("./simulation/imav_2026.sdf")
root = tree.getroot()

boxes = ["M3Box1", "M3Box2", "M3Box3"]
chosen = random.choice(boxes)

for model in root.iter("model"):
    if model.get("name") == chosen:
        visual = model.find(".//visual")

        plugin = ET.Element("plugin", {
            "filename": "gz-sim-thermal-system",
            "name": "gz::sim::systems::Thermal"
        })

        temp = ET.SubElement(plugin, "temperature")
        temp.text = "372.15"

        visual.append(plugin)

tree.write("./simulation/imav_2026_random.sdf")
print("done")