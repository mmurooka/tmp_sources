import h5py
import os
import sys

def compress_hdf5(input_file, output_file, compression="gzip", compression_opts=9):
    """
    HDF5ファイルを指定の圧縮アルゴリズムで圧縮し、属性を保持する。
    
    Args:
        input_file (str): 元のHDF5ファイルのパス。
        output_file (str): 圧縮後のHDF5ファイルのパス。
        compression (str): 圧縮アルゴリズム（デフォルト: "gzip"）。
        compression_opts (int): 圧縮レベル（1-9, gzipの場合）。
    """
    with h5py.File(input_file, "r") as src, h5py.File(output_file, "w") as dst:
        def copy_group(src_group, dst_group):
            """
            グループとその内容を再帰的にコピーし、属性も保持する。
            """
            # グループの属性をコピー
            for attr_name, attr_value in src_group.attrs.items():
                dst_group.attrs[attr_name] = attr_value

            # 各キーを処理
            for key, item in src_group.items():
                if isinstance(item, h5py.Dataset):
                    # データセットのコピー（圧縮適用）
                    dst_group.create_dataset(
                        key,
                        data=item[()],
                        compression=compression,
                        compression_opts=compression_opts
                    )
                    # データセットの属性をコピー
                    for attr_name, attr_value in item.attrs.items():
                        dst_group[key].attrs[attr_name] = attr_value
                elif isinstance(item, h5py.Group):
                    # サブグループの再帰的コピー
                    new_group = dst_group.create_group(key)
                    copy_group(item, new_group)

        # トップレベルグループをコピー
        copy_group(src, dst)

    print(f"Compressed file saved as: {output_file}")

# コマンドライン引数を処理
if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python compress_hdf5.py <input_file> <output_file>")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]

    if not os.path.exists(input_file):
        print(f"Error: Input file '{input_file}' does not exist.")
        sys.exit(1)

    compress_hdf5(input_file, output_file)
