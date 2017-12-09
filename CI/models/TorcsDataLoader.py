import torch
import pandas as pd
import os
import numpy as np
from torch.utils.data import Dataset, DataLoader

path_to_project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
data_dir = os.path.join(path_to_project_root, 'data')

class TorcsTrackDataset(Dataset):
    """Represents data from racing a particular track in TORCS"""

    def __init__(self, csv_files):
        data = []
        for file_name in csv_files:
            track = pd.read_csv(os.path.join(data_dir, file_name), index_col=None, header=0)
            data.append(track)

        data = pd.concat(data)

        self.carstates = data.iloc[:, 3:].values
        self.x_dim = self.carstates.shape[1]

        self.targets = data.iloc[:, 2].values

    def __len__(self):
        return len(self.track_frame)

    def __getitem__(self, idx):
        datapoint = self.carstates[idx]
        target = self.targets[idx]

        return {'input' : datapoint, 'target' : target}

class AalborgDataset(TorcsTrackDataset):
    def __init__(self):
        path = os.path.join(*PATH_AALBORG)
        super().__init__(path)

def save_parameters(model, label):
    in_dim, hidden, out_dim = model.get_n_units()

    saved_models = os.path.join(path_to_project_root, 'saved_models')
    model_name = str(type(model)).split('.')[-1]
    print(model_name)
    model_name = model_name[:-2]

    torch.save(model.state_dict(), os.path.join(saved_models,
                                    '{}_{}-{}-{}_{}.h5'.format(label, in_dim, hidden, out_dim, model_name)))
