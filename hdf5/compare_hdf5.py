import sys
import h5py
import numpy as np

def compare_hdf5_files(file1, file2):
    """
    2つのHDF5ファイルが再帰的に同じ内容を持つか確認する。
    
    Args:
        file1 (str): 元のHDF5ファイルのパス。
        file2 (str): 圧縮後のHDF5ファイルのパス。
    
    Returns:
        bool: 内容が一致すればTrue、一致しなければFalse。
    """
    def compare_groups(group1, group2, path=""):
        """
        2つのHDF5グループが再帰的に一致するか確認する。
        
        Args:
            group1: HDF5ファイルのグループ1。
            group2: HDF5ファイルのグループ2。
            path: 現在のパス（ログ用）。
            
        Returns:
            bool: 一致すればTrue、一致しなければFalse。
        """
        # キー（データセット、グループ）の比較
        keys1 = set(group1.keys())
        keys2 = set(group2.keys())
        if keys1 != keys2:
            print(f"Key mismatch at path '{path}': {keys1} != {keys2}")
            return False

        # 各キーについて比較
        for key in keys1:
            item1 = group1[key]
            item2 = group2[key]
            current_path = f"{path}/{key}"

            # データセットの比較
            if isinstance(item1, h5py.Dataset) and isinstance(item2, h5py.Dataset):
                if not np.array_equal(item1[()], item2[()]):
                    print(f"Dataset mismatch at path '{current_path}'")
                    return False

            # グループの比較（再帰）
            elif isinstance(item1, h5py.Group) and isinstance(item2, h5py.Group):
                if not compare_groups(item1, item2, current_path):
                    return False

            # 型が異なる場合
            else:
                print(f"Type mismatch at path '{current_path}': {type(item1)} != {type(item2)}")
                return False

        # 属性の比較
        if not compare_attributes(group1, group2, path):
            return False

        return True

    def compare_attributes(item1, item2, path=""):
        """
        2つのHDF5オブジェクトの属性が一致するか確認する。
        
        Args:
            item1: HDF5ファイルのオブジェクト1。
            item2: HDF5ファイルのオブジェクト2。
            path: 現在のパス（ログ用）。
            
        Returns:
            bool: 一致すればTrue、一致しなければFalse。
        """
        attrs1 = dict(item1.attrs)
        attrs2 = dict(item2.attrs)
        if attrs1 != attrs2:
            print(f"Attribute mismatch at path '{path}': {attrs1} != {attrs2}")
            return False
        return True

    # ファイルを開いて比較
    with h5py.File(file1, "r") as f1, h5py.File(file2, "r") as f2:
        return compare_groups(f1, f2)

# 実行例
if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python compare_hdf5.py <input_file> <output_file>")
        sys.exit(1)

    file1 = sys.argv[1]
    file2 = sys.argv[2]

    if compare_hdf5_files(file1, file2):
        print("The files are identical.")
    else:
        print("The files are NOT identical.")
