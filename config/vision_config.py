from pathlib import Path

import torch

BASE_DIR = Path(__file__).resolve().parent.parent

# Model paths for PyTorch
BUOY_MODEL_PATH = str(BASE_DIR / "models" / "buoy_model.pt")
VESSEL_MODEL_PATH = str(BASE_DIR / "models" / "vessel_model.pt")

# Device selection for PyTorch
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
