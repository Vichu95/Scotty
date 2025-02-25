import dask.dataframe as dd
import pandas as pd
import random
from bokeh.plotting import figure, curdoc
from bokeh.models import ColumnDataSource, MultiSelect, CheckboxGroup, Button, HoverTool, Legend
from bokeh.layouts import column, row

# Load CSV with Dask (Efficient for Large Files)
file_path = "merged_log.csv"
df = dd.read_csv(file_path, low_memory=False).compute()  # Load only required parts

# Clean column names (remove leading spaces)
df.columns = df.columns.str.strip()

# **Automatically Detect All Numeric Signal Columns (No Hardcoding!)**
excluded_columns = ["Leg_Index"]  # Exclude non-numeric columns
signal_columns = [col for col in df.columns if col not in excluded_columns and df[col].dtype != "object"]

# Leg Selection Mapping (Short Names)
leg_mapping = {
    0: "FR",  # Front Right
    1: "FL",  # Front Left
    2: "BR",  # Back Right
    3: "BL"   # Back Left
}

# **Step 1: Split Data by Leg**
leg_data = {}
for leg in leg_mapping.keys():
    leg_df = df[df["Leg_Index"] == leg].copy()
    leg_data[leg] = leg_df.reset_index(drop=True)  # Store data per leg

# **Step 2: Ensure All Legs Have the Same Size**
min_size = min([len(leg_data[l]) for l in leg_data])  # Find smallest dataset size
for leg in leg_data:
    leg_data[leg] = leg_data[leg].iloc[:min_size]  # Trim to match smallest dataset

# UI Components
leg_select = CheckboxGroup(labels=[leg_mapping[i] for i in range(4)], active=[])  # No legs selected initially
signal_select = MultiSelect(title="Select Signals", options=signal_columns, value=[], size=len(signal_columns))  # Full height dropdown
plot_button = Button(label="Plot", button_type="primary")
reset_button = Button(label="Reset", button_type="danger")

# **Create Empty ColumnDataSource**
source = ColumnDataSource(data={'x': []})

# **Create Figure (Full Width for Large Screen)**
p = figure(title="",
           x_axis_label="Samples",
           y_axis_label="Signal Value",
           sizing_mode="stretch_both",  # Automatically fills the screen
           tools="pan,wheel_zoom,box_zoom,reset,save",
           active_scroll="wheel_zoom")

# **Add Hover Tool (Fixing `???` issue)**
hover = HoverTool(tooltips=[("Index", "@x"), ("Signal", "$name"), ("Value", "@$name")])
p.add_tools(hover)

# **Store Active Renderers (to fully remove on reset)**
active_renderers = []

# **Dark Color Palette for Better Visibility**
dark_colors = ["#E63946", "#457B9D", "#2A9D8F", "#F4A261", "#8D99AE", "#6D597A", "#FFB703", "#264653", "#E76F51"]
signal_color_map = {}

# **Function to Assign Colors Based on Signal Count**
def get_signal_colors(num_signals):
    return dark_colors[:num_signals]  # Pick first N colors from the dark palette

# **Function to Plot Data When "Plot" is Clicked**
def plot_data():
    global active_renderers, signal_color_map  # Keep track of active signals

    # **Clear Previous Data & Reset Source**
    source.data = {'x': []}  # Empty the source completely
    p.renderers = []  # Remove old signals
    active_renderers = []  # Reset active renderers list
    signal_color_map.clear()  # Reset color mapping

    # **Get Selected Legs**
    selected_legs = [list(leg_mapping.keys())[i] for i in leg_select.active]

    if not selected_legs:
        return  # No legs selected, exit function

    # **Get Selected Signals**
    selected_signals = signal_select.value
    num_selected_signals = len(selected_signals) * len(selected_legs)

    # **Assign Colors Dynamically Based on Signal Count**
    colors = get_signal_colors(num_selected_signals)

    # **Create a new data dictionary with x-axis values**
    new_data = {'x': list(range(min_size))}  # X-axis is based on row index

    # **Update DataSource and Add to Plot**
    color_index = 0
    legend_items = []
    for leg in selected_legs:
        leg_df = leg_data[leg]  # Get preprocessed data for the selected leg

        for signal in selected_signals:
            key = f"{leg_mapping[leg]}_{signal}"  # Unique legend name

            # Store separate data for each leg-signal combination
            new_data[key] = leg_df[signal].tolist()  # Convert to list for consistency

            # **Assign a unique color from the updated dark color palette**
            color = colors[color_index % len(colors)]
            signal_color_map[key] = color
            color_index += 1

            # **Add new line plot**
            line = p.line('x', key, source=source, legend_label=key, line_width=2, color=color, name=key)
            active_renderers.append(line)  # Track active renderers
            legend_items.append((key, [line]))

    # **Update Bokeh DataSource**
    source.data = new_data

    # **Fix: Reset and Refresh Legend Properly**
    p.legend.items = legend_items  # Rebuild legend from active signals
    # p.legend.title = "Active Signals"
    p.legend.click_policy = "hide"

# **Fix: Reset Function Now Completely Clears Everything**
def reset_plot():
    global active_renderers, signal_color_map

    # **Clear Data Source**
    source.data = {'x': []}

    # **Clear Renderers & Active Plots**
    p.renderers = []
    active_renderers = []  # Reset tracked renderers

    # **Clear User Selections**
    leg_select.active = []
    signal_select.value = []

    # **Clear Color Mappings**
    signal_color_map.clear()

    # **Clear Legend Properly**
    p.legend.items = []  # Removes old legend entries

# **Attach Button Click Events**
plot_button.on_click(plot_data)
reset_button.on_click(reset_plot)

# **Final Layout: Sidebar (Left) + Plot (Right)**
layout = row(
    column(leg_select, signal_select, plot_button, reset_button, width=350, sizing_mode="stretch_height"),  # Sidebar with full dropdown
    p,  # Full-width Plot
    sizing_mode="stretch_both"  # Ensures everything resizes properly
)

curdoc().add_root(layout)
curdoc().title = "Scotty Log Viewer"
