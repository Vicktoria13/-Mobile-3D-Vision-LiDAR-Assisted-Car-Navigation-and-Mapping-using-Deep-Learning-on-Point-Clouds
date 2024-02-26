import numpy as np
from tiler import Tiler, Merger

image = np.random.random((3, 1920, 1080))

# Setup tiling parameters
tiler = Tiler(data_shape=image.shape,
              tile_shape=(3, 250, 250),
              channel_dimension=0)

## Access tiles:
# 1. with an iterator
for tile_id, tile in tiler.iterate(image):
   print(f'Tile {tile_id} out of {len(tiler)} tiles.')
# 1b. the iterator can also be accessed through __call__
for tile_id, tile in tiler(image):
   print(f'Tile {tile_id} out of {len(tiler)} tiles.')
# 2. individually
tile_3 = tiler.get_tile(image, 3)
# 3. in batches
tiles_in_batches = [batch for _, batch in tiler(image, batch_size=10)]

# Setup merging parameters
merger = Merger(tiler)

## Merge tiles:
# 1. one by one

merger.reset()
for batch_id, batch in tiler(image, batch_size=10):
   merger.add_batch(batch_id, 10, batch)

# Final merging: applies tapering and optional unpadding
final_image = merger.merge(unpad=True)  # (3, 1920, 1080)
