#!/usr/bin/env python3
"""Isolated PyTorch SAM 2 worker; no ROS/control/simulator truth access."""
import argparse
import base64
import contextlib
import json
import sys
import time
import zlib

import cv2
import numpy as np
import torch

from wholebody_catch.generic_perception import mask_grasp


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--vendor', required=True)
    parser.add_argument('--weights', required=True)
    args = parser.parse_args()
    sys.path.insert(0, args.vendor)
    torch.set_num_threads(2)
    with contextlib.redirect_stdout(sys.stderr):
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor
        from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        model = build_sam2('configs/sam2.1/sam2.1_hiera_t.yaml', args.weights,
                           device=device, apply_postprocessing=False)
        predictor = SAM2ImagePredictor(model)
        generator = SAM2AutomaticMaskGenerator(model, points_per_side=24, points_per_batch=64,
                                               pred_iou_thresh=.8, stability_score_thresh=.90)
    print(json.dumps(dict(ready=True, device=device)), flush=True)
    tracked = None
    misses = 0
    for line in sys.stdin:
        try:
            request = json.loads(line)
            bgr = cv2.imdecode(np.frombuffer(base64.b64decode(request['image']), np.uint8), cv2.IMREAD_COLOR)
            depth = np.frombuffer(zlib.decompress(base64.b64decode(request['depth'])), '<f4').reshape(bgr.shape[:2])
            T = np.array(request['transform']).reshape(4, 4)
            K = np.array(request['K']).reshape(3, 3)
            if request.get('reset'):
                tracked = None
            start = time.perf_counter()
            with torch.inference_mode(), torch.autocast(device_type=device, dtype=torch.bfloat16, enabled=device == 'cuda'), contextlib.redirect_stdout(sys.stderr):
                candidates = []
                if tracked is not None:
                    optical = T[:3, :3].T@(tracked-T[:3, 3])
                    uv = (K@optical)[:2]/max(optical[2], 1e-6)
                    if optical[2] > .025 and 0 <= uv[0] < bgr.shape[1] and 0 <= uv[1] < bgr.shape[0]:
                        predictor.set_image(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
                        masks, scores, _ = predictor.predict(point_coords=uv[None], point_labels=np.array([1]),
                                                              multimask_output=True)
                        candidates = [(m, float(s)) for m, s in zip(masks, scores) if s > .7]
                else:
                    candidates = [(m['segmentation'], float(m['predicted_iou']))
                                  for m in generator.generate(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))]
                proposals = []
                for mask, confidence in candidates:
                    grasp = mask_grasp(mask, depth, K, T, request['workspace_min'], request['workspace_max'],
                                       request['max_width'], request['insertion'])
                    if grasp is not None:
                        grasp['confidence'] = confidence
                        if tracked is None or np.linalg.norm(np.array(grasp['point'])-tracked) < .06:
                            proposals.append(grasp)
                if proposals:
                    proposals.sort(key=lambda g: np.linalg.norm(np.array(g['point'])-(tracked if tracked is not None else T[:3, 3])))
                    selected = proposals[0]
                    tracked = np.array(selected['point'])
                    misses = 0
                else:
                    selected = None
                    misses += 1
                    # Keep identity during loss; a fresh search is explicit.
            print(json.dumps(dict(candidate=selected, candidates=len(proposals), misses=misses,
                                  inference_ms=1000*(time.perf_counter()-start))), flush=True)
        except Exception as exc:
            print(json.dumps(dict(error=str(exc))), flush=True)


if __name__ == '__main__':
    main()
