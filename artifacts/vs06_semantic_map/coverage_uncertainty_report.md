# Coverage and uncertainty report

Observed coverage: limited to replay-sampled map/camera/lidar evidence and free map cells; this does not claim every shoe.
Inaccessible regions: occupied, unknown, or unreachable cells are excluded from semantic absence claims.
Occlusion: shoes hidden behind obstacles, outside the camera view, or below reliable ground-plane projection remain unverified.
Uncertain detections: review-level tracks are preserved as candidates with explicit status and evidence references.
Detector confidence and projection confidence are reported separately; combined confidence is conservative.
No bogus every-shoe claim: the artifacts avoid completeness claims outside observed coverage.

## Counts
- Semantic tracks: 1
- Semantic observations: 4
- Detector candidates: 60
- Detector accepted count: 0
- Detector coverage statement: Replay evaluation is limited to the sampled/available frames; do not infer every-shoe recall. This primary replay bag currently provides negative/no-positive coverage for shoes.
- Map cell counts: {'free': 24, 'occupied': 28, 'unknown': 12, 'total': 64}

## Regions
- observed: Map cells and sampled camera frames covered by the replay pipeline (confidence 0.375)
- inaccessible: Unknown, occupied, or unreachable map regions were not searched (confidence 0.0)
- occluded: Objects hidden by obstacles or outside line of sight remain unverified (confidence 0.0)
