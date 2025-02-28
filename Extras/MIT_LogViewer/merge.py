import pandas as pd

# Load both CSV files
# folderpath="C:\\Users\\Vishnudev K\\Nextcloud\\DataV_Thesis\\Scotty\\01_Documentation\\01_Report\\Gallery\\Feb27"
folderpath="D:\\Learn\\Anhalt\\0_Study\\5_Thesis\\Scotty\\01_Documentation\\01_Report\\Gallery\\Feb27"
timestamp = '202502271330'
command_file = folderpath + "\\" + "spi_command_log_" + timestamp + ".csv"
data_file = folderpath + "\\" + "spi_data_log_" + timestamp + ".csv"
stm_cmd_file = folderpath + "\\" + "stm_command_" + timestamp + ".csv"

# Read CSVs
command_df = pd.read_csv(command_file)
data_df = pd.read_csv(data_file)
stm_cmd_df = pd.read_csv(stm_cmd_file)

# Ensure Columns Are Named Uniquely (Prefixing)
command_df = command_df.add_prefix("cmd_")  # Prefix all command columns with "cmd_"
data_df = data_df.add_prefix("data_")  # Prefix all data columns with "data_"
stm_cmd_df = stm_cmd_df.add_prefix("stm_")

# Restore `Leg_Index` (remove prefix from this specific column)
command_df.rename(columns={"cmd_Leg_Index": "Leg_Index"}, inplace=True)
data_df.rename(columns={"data_Leg_Index": "Leg_Index"}, inplace=True)
stm_cmd_df.rename(columns={"stm_Leg_Index": "Leg_Index"}, inplace=True)

# **Check if all logs have the same number of rows**
min_rows = min(len(command_df), len(data_df), len(stm_cmd_df))
if len(command_df) != len(data_df) or len(command_df) != len(stm_cmd_df):
    print(f"WARNING: Logs have different lengths! Trimming to {min_rows} rows.")
    command_df = command_df.iloc[:min_rows]
    data_df = data_df.iloc[:min_rows]
    stm_cmd_df = stm_cmd_df.iloc[:min_rows]

# **Merge all logs row by row while keeping Leg_Index**
merged_df = pd.concat([command_df, data_df.drop(columns=["Leg_Index"]), stm_cmd_df.drop(columns=["Leg_Index"])], axis=1)


# Save to a new CSV file
merged_file = "merged_log.csv"
merged_df.to_csv(merged_file, index=False)

print(f"Merged log saved as: {merged_file}")
