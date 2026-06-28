# A single consolidated object in the spatial semantic map.
# All spatial quantities are expressed in the map frame (see pose.header.frame_id).

uint32 id                          # stable identifier within one semantic map
string label                       # canonical / primary label, e.g. "chair"
string[] aliases                   # other labels observed for this same entity
string description                 # rich, VLM-generated natural-language description
string region                      # room / zone name (e.g. "kitchen"); may be empty

geometry_msgs/PoseStamped pose     # map-frame centroid; orientation optional
geometry_msgs/Vector3 size         # axis-aligned extent estimate (m); may be zero

float32 confidence                 # [0,1] aggregate detection confidence
uint32 observation_count           # number of fused observations behind this entity
builtin_interfaces/Time last_seen  # stamp of the most recent observation

float32[] embedding                # semantic embedding (CLIP/text); may be empty over the wire
