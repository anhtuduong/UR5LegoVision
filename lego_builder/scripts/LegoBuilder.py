"""!
@file LegoBuilder.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@date 2023-11-17

@brief Defines the app LegoBuilder.
"""
import os
import sys
from pathlib import Path

# Resolve paths
FILE = Path(__file__).resolve()
ROOT = FILE.parents[2]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

BUTTON_IMG_PATH = os.path.abspath(os.path.join(ROOT, "lego_builder/block_img"))
OUTPUT_PATH = os.path.abspath(os.path.join(ROOT, "lego_builder/output"))

CANVAS_WIDTH = 500
CANVAS_HEIGHT = 500
BUTTON_IMG_PX = 40
IMG_PX = 200
SCALE_FACTOR = 2
EXPORT_TRANSPARENCY = True

import tkinter as tk
from PIL import Image, ImageTk
from constants import BLOCK_NAMES

class BlockImage:
    def __init__(self, pil_image, tk_image):
        self.pil_image = pil_image
        self.tk_image = tk_image

class LegoBuilder:
    def __init__(self, root):
        self.root = root
        self.root.title("Lego Builder")

        self.current_block = None
        self.ghost_block = None

        self.create_widgets()

    def create_widgets(self):
        # Create a frame to contain the buttons with a scrollbar
        button_frame = tk.Frame(self.root)
        button_frame.pack(side=tk.LEFT, padx=10, pady=10, fill=tk.Y)

        # Add a canvas to place the buttons and a scrollbar
        canvas = tk.Canvas(button_frame)
        scrollbar = tk.Scrollbar(button_frame, orient=tk.VERTICAL, command=canvas.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.pack(side=tk.LEFT, padx=5, pady=5, fill=tk.BOTH, expand=True)
        canvas.configure(yscrollcommand=scrollbar.set)

        # Create a frame inside the canvas to hold the buttons
        frame_buttons = tk.Frame(canvas)
        canvas.create_window((0, 0), window=frame_buttons, anchor=tk.NW)

        # Bind the canvas to handle resizing of the buttons
        frame_buttons.bind("<Configure>", lambda event, canvas=canvas: self.on_configure(event, canvas))

        # Bind the mouse wheel to the button_frame for scrolling
        button_frame.bind("<MouseWheel>", lambda event, canvas=canvas: self.on_mousewheel(event, canvas))

        # Create the buttons
        block_names = BLOCK_NAMES
        self.block_buttons = []
        buttons_per_row = 2  # Number of buttons per row

        for idx, block_name in enumerate(block_names):
            if idx % buttons_per_row == 0:
                button_row = tk.Frame(frame_buttons)
                button_row.pack(side=tk.TOP)

            block_image = Image.open(f"{BUTTON_IMG_PATH}/{block_name}.png")
            button_block_img = block_image.copy()
            button_block_img = button_block_img.resize((BUTTON_IMG_PX, BUTTON_IMG_PX), Image.LANCZOS)
            button_tk_image = ImageTk.PhotoImage(button_block_img)

            block_button = tk.Button(button_row, text=block_name, image=button_tk_image, compound=tk.TOP,
                                    command=lambda img=block_image.copy(): self.select_block(img))
            block_button.image = button_tk_image
            block_button.pack(side=tk.LEFT, padx=5, pady=5)

            self.block_buttons.append(block_button)

        self.canvas = tk.Canvas(self.root, width=CANVAS_WIDTH, height=CANVAS_HEIGHT, bg="white")
        self.canvas.pack(side=tk.LEFT, padx=10, pady=10)
        self.canvas.bind("<Button-1>", self.place_block)
        self.canvas.bind("<Button-3>", self.delete_block)

        # Create frame for right-side buttons
        right_button_frame = tk.Frame(self.root)
        right_button_frame.pack(side=tk.LEFT, padx=1, pady=10, anchor=tk.NE)

        rotate_button = tk.Button(right_button_frame, text="Rotate", command=self.rotate_block)
        rotate_button.pack(side=tk.TOP, padx=10, pady=5)

        flip_horizontal_button = tk.Button(right_button_frame, text="Flip Horizontal", command=self.flip_horizontal)
        flip_horizontal_button.pack(side=tk.TOP, padx=10, pady=5)

        flip_vertical_button = tk.Button(right_button_frame, text="Flip Vertical", command=self.flip_vertical)
        flip_vertical_button.pack(side=tk.TOP, padx=10, pady=5)

        export_button = tk.Button(self.root, text="Export Image", command=self.export_image)
        export_button.pack(side=tk.BOTTOM, padx=10, pady=10)

        self.canvas_items = {}

    def on_configure(self, event, canvas):
        canvas.configure(scrollregion=canvas.bbox("all"))

    def on_mousewheel(self, event, canvas):
        canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def select_block(self, block_image):
        block_image = block_image.resize((IMG_PX, IMG_PX), Image.LANCZOS)
        pil_image = block_image.copy()  # Make a copy of the PIL image
        tk_image = ImageTk.PhotoImage(block_image)
        self.current_block = tk_image
        self.pil_image = pil_image
        self.canvas.delete("ghost_block")  # Remove previous ghost block if exists

        # Bind motion to update ghost block position within canvas
        self.canvas.bind("<Motion>", self.update_ghost_block_position)

    def update_ghost_block_position(self, event):
        if self.is_within_canvas(event):
            x, y = event.x, event.y
            self.canvas.delete("ghost_block")  # Remove previous ghost block if exists

            block_width = self.current_block.width()  # Get width of the block image
            block_height = self.current_block.height()  # Get height of the block image

            x -= block_width / 2  # Adjust x-coordinate to center the image
            y -= block_height / 2  # Adjust y-coordinate to center the image

            self.ghost_block = self.canvas.create_image(x, y, anchor=tk.NW, image=self.current_block, tags="ghost_block")

    def is_within_canvas(self, event):
        x, y = event.x, event.y
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()

        return 0 <= x <= canvas_width and 0 <= y <= canvas_height

    def place_block(self, event):
        if self.current_block is not None and self.pil_image is not None:
            x, y = event.x, event.y

            block_image = BlockImage(self.pil_image, self.current_block)
            block_width = self.current_block.width()  # Get width of the block image
            block_height = self.current_block.height()  # Get height of the block image
            x -= block_width / 2  # Adjust x-coordinate to center the image
            y -= block_height / 2  # Adjust y-coordinate to center the image
            self.canvas_items[self.canvas.create_image(x, y, anchor=tk.NW, image=self.current_block, tags="added_block")] = block_image

    def rotate_block(self):
        if self.ghost_block:
            self.canvas.delete(self.ghost_block)
            self.pil_image = self.pil_image.rotate(90)
            self.current_block = ImageTk.PhotoImage(self.pil_image)
            self.ghost_block = self.canvas.create_image(0, 0, anchor=tk.NW, image=self.current_block, tags="ghost_block")
            self.canvas.tag_lower(self.ghost_block)  # Ensure ghost block stays at the bottom

    def flip_horizontal(self):
        if self.ghost_block:
            self.canvas.delete(self.ghost_block)
            self.pil_image = self.pil_image.transpose(Image.FLIP_LEFT_RIGHT)
            self.current_block = ImageTk.PhotoImage(self.pil_image)
            self.ghost_block = self.canvas.create_image(0, 0, anchor=tk.NW, image=self.current_block, tags="ghost_block")
            self.canvas.tag_lower(self.ghost_block)

    def flip_vertical(self):
        if self.ghost_block:
            self.canvas.delete(self.ghost_block)
            self.pil_image = self.pil_image.transpose(Image.FLIP_TOP_BOTTOM)
            self.current_block = ImageTk.PhotoImage(self.pil_image)
            self.ghost_block = self.canvas.create_image(0, 0, anchor=tk.NW, image=self.current_block, tags="ghost_block")
            self.canvas.tag_lower(self.ghost_block)

    def delete_block(self, event):
        items = self.canvas.find_closest(event.x, event.y)
        if "ghost_block" not in self.canvas.gettags(items):
            self.canvas.delete(items)

    def export_image(self):
        canvas_width = CANVAS_WIDTH * SCALE_FACTOR
        canvas_height = CANVAS_HEIGHT * SCALE_FACTOR
        canvas_image = Image.new("RGBA", (canvas_width, canvas_height), (255, 255, 255, 0))

        # Get the canvas background and paste it onto the canvas image
        if EXPORT_TRANSPARENCY:
            canvas_bg = Image.new("RGBA", (canvas_width, canvas_height), (255, 255, 255, 0))
        else:
            canvas_bg = Image.new("RGB", (canvas_width, canvas_height), "white")
        canvas_image.paste(canvas_bg, (0, 0))

        # Iterate through canvas items and export them onto the canvas image
        for item, block_image in self.canvas_items.items():
            x0, y0 = self.canvas.coords(item)
            x0 *= SCALE_FACTOR
            y0 *= SCALE_FACTOR
            block_image = block_image.pil_image
            block_image = block_image.resize((IMG_PX * SCALE_FACTOR, IMG_PX * SCALE_FACTOR), Image.LANCZOS)

            # Get the alpha channel from the block image
            alpha = block_image.getchannel('A')

            # Paste the block image onto the canvas image considering the alpha channel
            canvas_image.paste(block_image, (int(x0), int(y0)), mask=alpha)

        # Save the canvas image with transparency and background intact
        canvas_image.save(f"{OUTPUT_PATH}/output.png")
        print("Image exported successfully.")


def main():
    root = tk.Tk()
    app = LegoBuilder(root)
    root.mainloop()

if __name__ == "__main__":
    main()
