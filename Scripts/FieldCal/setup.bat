REM Have had issues with .venv not created in the repo root, so put it there!
cd ../..
pip install uv
uv venv
uv pip install -r Scripts/FieldCal/requirements.txt
