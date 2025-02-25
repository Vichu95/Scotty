import pandas as pd

# Load both CSV files
timestamp = '202502241221'
command_file = "spi_command_log_" + timestamp + ".csv"
data_file = "spi_data_log_" + timestamp + ".csv"

# Read CSVs
command_df = pd.read_csv(command_file)
data_df = pd.read_csv(data_file)

# Ensure Columns Are Named Uniquely (Prefixing)
command_df = command_df.add_prefix("cmd_")  # Prefix all command columns with "cmd_"
data_df = data_df.add_prefix("data_")  # Prefix all data columns with "data_"

# Restore `Leg_Index` (remove prefix from this specific column)
command_df.rename(columns={"cmd_Leg_Index": "Leg_Index"}, inplace=True)
data_df.rename(columns={"data_Leg_Index": "Leg_Index"}, inplace=True)

# **Check if both logs have the same number of rows**
if len(command_df) != len(data_df):
    print(f"WARNING: Command log ({len(command_df)}) and Data log ({len(data_df)}) have different lengths!")
    min_rows = min(len(command_df), len(data_df))
    command_df = command_df.iloc[:min_rows]  # Trim to the smaller size
    data_df = data_df.iloc[:min_rows]  # Trim to the smaller size

# **Merge row by row while keeping Leg_Index order intact**
merged_df = pd.concat([command_df, data_df.drop(columns=["Leg_Index"])], axis=1)  # Drop redundant Leg_Index column

# Save to a new CSV file
merged_file = "merged_log.csv"
merged_df.to_csv(merged_file, index=False)

print(f"Merged log saved as: {merged_file}")
