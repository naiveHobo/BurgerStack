# A single per-frame object detection, already localized in 3D.
# Produced by semantic_perception; consumed by semantic_mapping (one Detection ->
# one Observation in the spatial store). This is a raw sighting, NOT a consolidated
# entity (see SemanticObject for the consolidated form).

std_msgs/Header header        # stamp = source image stamp; frame_id = frame of `position` (normally map)
string label
float32 confidence            # [0, 1]
geometry_msgs/Point position  # 3D position in header.frame_id

uint32 x                      # source-image 2D bounding box, pixels
uint32 y
uint32 width
uint32 height

sensor_msgs/Image crop        # cropped RGB region, kept for batch enrichment (VLM + embedding); may be empty
