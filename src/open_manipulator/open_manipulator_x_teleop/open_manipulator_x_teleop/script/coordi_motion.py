import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk


def calculate_homography(shear_factor, rotation_angle):
    shear_matrix = np.array([[1, shear_factor], [0, 1]])
    theta = np.radians(rotation_angle)
    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    homography_matrix = rotation_matrix.dot(shear_matrix)
    return homography_matrix


def update_plot(shear_factor, rotation_angle):
    shear_factor = float(shear_factor)
    rotation_angle = float(rotation_angle)

    homography_matrix = calculate_homography(shear_factor, rotation_angle)
    inv_homography_matrix = np.linalg.inv(homography_matrix)

    original_coords_x = np.vstack((x, y_x))
    original_coords_y = np.vstack((x_y, y))

    sheared_coords_x = homography_matrix.dot(original_coords_x)
    sheared_coords_y = homography_matrix.dot(original_coords_y)

    ax.clear()
    ax.plot(x, y_x, 'bo-', label='Original X Parallel')
    ax.plot(x_y, y, 'go-', label='Original Y Parallel')
    ax.plot(sheared_coords_x[0, :], sheared_coords_x[1, :], 'ro-', label='Transformed X Parallel')
    ax.plot(sheared_coords_y[0, :], sheared_coords_y[1, :], 'mo-', label='Transformed Y Parallel')
    ax.axhline(0, color='gray', lw=0.5)
    ax.axvline(0, color='gray', lw=0.5)
    ax.legend()
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_title('Shear and Rotation Transformation')
    ax.grid(True)

    canvas.draw()

    homography_text.set(
        f"Homography Matrix:\n{homography_matrix}\n\nInverse Homography Matrix:\n{inv_homography_matrix}")


def on_shear_slider_change(value):
    shear_value.set(f"{float(value):.2f}")
    update_plot(value, rotation_slider.get())


def on_rotation_slider_change(value):
    rotation_value.set(f"{float(value):.2f}")
    update_plot(shear_slider.get(), value)


def on_shear_entry_change(*args):
    try:
        value = float(shear_value.get())
        shear_slider.set(value)
        update_plot(value, rotation_slider.get())
    except ValueError:
        pass


def on_rotation_entry_change(*args):
    try:
        value = float(rotation_value.get())
        rotation_slider.set(value)
        update_plot(shear_slider.get(), value)
    except ValueError:
        pass


# 예제 데이터
x = np.array([0, 1, 2, 3])
y_x = np.array([0, 0, 0, 0])
x_y = np.array([0, 0, 0, 0])
y = np.array([0, 1, 2, 3])

# Tkinter GUI 설정
root = tk.Tk()
root.title("Shear and Rotation Transformation")

mainframe = ttk.Frame(root, padding="10 10 20 20")
mainframe.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

# Shear factor slider
shear_label = ttk.Label(mainframe, text="Shear Factor")
shear_label.grid(column=1, row=1, sticky=tk.W)
shear_slider = ttk.Scale(mainframe, from_=-2.0, to=2.0, orient=tk.HORIZONTAL, command=on_shear_slider_change)
shear_slider.set(0.5)
shear_slider.grid(column=2, row=1, sticky=(tk.W, tk.E))

shear_value = tk.StringVar(value=f"{shear_slider.get():.2f}")
shear_entry = ttk.Entry(mainframe, textvariable=shear_value, width=5)
shear_entry.grid(column=3, row=1, sticky=tk.W)
shear_value.trace('w', on_shear_entry_change)

# Rotation angle slider
rotation_label = ttk.Label(mainframe, text="Rotation Angle")
rotation_label.grid(column=1, row=2, sticky=tk.W)
rotation_slider = ttk.Scale(mainframe, from_=-180, to=180, orient=tk.HORIZONTAL, command=on_rotation_slider_change)
rotation_slider.set(30)
rotation_slider.grid(column=2, row=2, sticky=(tk.W, tk.E))

rotation_value = tk.StringVar(value=f"{rotation_slider.get():.2f}")
rotation_entry = ttk.Entry(mainframe, textvariable=rotation_value, width=5)
rotation_entry.grid(column=3, row=2, sticky=tk.W)
rotation_value.trace('w', on_rotation_entry_change)

# Matplotlib Figure
fig, ax = plt.subplots(figsize=(6, 6))
canvas = FigureCanvasTkAgg(fig, master=mainframe)
canvas.get_tk_widget().grid(column=1, row=3, columnspan=3)

# Homography matrix display
homography_text = tk.StringVar()
homography_label = ttk.Label(mainframe, textvariable=homography_text, justify=tk.LEFT)
homography_label.grid(column=1, row=4, columnspan=3, sticky=tk.W)
homography_text.set("Homography Matrix:\n")

# 초기 플롯 업데이트
update_plot(0.5, 30)

# Padding for all elements
for child in mainframe.winfo_children():
    child.grid_configure(padx=5, pady=5)

root.mainloop()