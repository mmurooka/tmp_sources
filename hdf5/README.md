# Utilities for HDF5 files

```console
$ python compress_hdf5.py original.hdf5 compressed.hdf5
Compressed file saved as: compressed.hdf5
# 3m26.064s

$ python compare_hdf5.py original.hdf5 compressed.hdf5
The files are identical.
# 28.092s

$ python decompress_hdf5.py compressed.hdf5 decompressed.hdf5
Decompressed file saved as: decompressed.hdf5
# 27.048s

$ du -h original.hdf5 compressed.hdf5 decompressed.hdf5 
5.8G	original.hdf5
1.4G	compressed.hdf5
5.8G	decompressed.hdf5
```
