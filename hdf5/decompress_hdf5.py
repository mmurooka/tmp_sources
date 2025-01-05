import h5py
import os
import sys

def decompress_hdf5(input_file, output_file):
    """
    圧縮されたHDF5ファイルを非圧縮で保存し、属性も保持する。
    
    Args:
        input_file (str): 圧縮されたHDF5ファイルのパス。
        output_file (str): 非圧縮で保存するHDF5ファイルのパス。
    """
    with h5py.File(input_file, "r") as src, h5py.File(output_file, "w") as dst:
        def copy_group(src_group, dst_group):
            """
            グループとその内容を再帰的にコピーし、非圧縮で保存。
            """
            # グループの属性をコピー
            for attr_name, attr_value in src_group.attrs.items():
                dst_group.attrs[attr_name] = attr_value

            # 各キーを処理
            for key, item in src_group.items():
                if isinstance(item, h5py.Dataset):
                    # データセットのコピー（非圧縮）
                    dst_group.create_dataset(
                        key,
                        data=item[()],
                        compression=None  # 非圧縮で保存
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

    print(f"Decompressed file saved as: {output_file}")

# コマンドライン引数を処理
if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python decompress_hdf5.py <input_file> <output_file>")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]

    if not os.path.exists(input_file):
        print(f"Error: Input file '{input_file}' does not exist.")
        sys.exit(1)

    decompress_hdf5(input_file, output_file)
