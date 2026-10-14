#!/usr/bin/env bash
# Install only task-local SAM 2 assets; preserve the existing PyTorch/CUDA stack.
set -euo pipefail
catch_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
catch_python="${CATCH_PYTHON:-$HOME/miniconda3/envs/torch/bin/python}"
catch_deps="$catch_ws/dependencies"
"$catch_python" - <<'PY'
import sys, torch, torchvision
assert sys.version_info[:2] == (3, 11), 'Use a Python 3.11 PyTorch environment for this dependency snapshot'
assert tuple(map(int, torch.__version__.split('+')[0].split('.')[:2])) >= (2, 5), 'SAM 2 requires PyTorch >= 2.5'
print('Existing runtime:', sys.executable, torch.__version__, 'CUDA available:', torch.cuda.is_available())
PY
mkdir -p "$catch_deps/catch_models" "$catch_deps/catch_python"
if [[ ! -d "$catch_deps/sam2/.git" ]]; then
  git clone https://github.com/facebookresearch/sam2.git "$catch_deps/sam2"
fi
catch_commit=2b90b9f5ceec907a1c18123530e92e794ad901a4
if [[ "$(git -C "$catch_deps/sam2" rev-parse HEAD)" != "$catch_commit" ]]; then
  git -C "$catch_deps/sam2" diff --quiet
  git -C "$catch_deps/sam2" diff --cached --quiet
  git -C "$catch_deps/sam2" fetch origin "$catch_commit"
  git -C "$catch_deps/sam2" checkout --detach "$catch_commit"
fi
touch "$catch_deps/sam2/COLCON_IGNORE"
"$catch_python" -m pip install --target "$catch_deps/catch_python" --no-deps \
  hydra-core==1.3.2 iopath==0.1.10 omegaconf==2.3.1 antlr4-python3-runtime==4.9.3 \
  portalocker==4.4.0 PyYAML==6.0.3 packaging==26.3 tqdm==4.70.1 typing_extensions==4.16.0
catch_weights="$catch_deps/catch_models/sam2.1_hiera_tiny.pt"
if [[ ! -f "$catch_weights" ]]; then
  curl --fail --location --retry 3 \
    https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt \
    --output "$catch_weights.partial"
  mv "$catch_weights.partial" "$catch_weights"
fi
printf '%s  %s\n' 7402e0d864fa82708a20fbd15bc84245c2f26dff0eb43a4b5b93452deb34be69 "$catch_weights" | sha256sum --check
PYTHONPATH="$catch_deps/catch_python:$catch_deps/sam2:$catch_ws/src/wholebody_catch" \
  "$catch_python" "$catch_ws/src/wholebody_catch/scripts/segment_worker.py" \
  --vendor "$catch_deps/sam2" --weights "$catch_weights" < /dev/null
echo 'SAM 2 assets ready. Rebuild wholebody_catch to install its launch resources.'
