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

BUTTON_IMG_PATH = os.path.abspath(os.path.join(ROOT, "lego_builder/button_img"))
OUTPUT_PATH = os.path.abspath(os.path.join(ROOT, "lego_builder/output"))
RESIZE_PX = 100

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
        block_frame = tk.Frame(self.root)
        block_frame.pack(side=tk.LEFT, padx=10, pady=10)

        block_names = BLOCK_NAMES
        self.block_buttons = []
        for block_name in block_names:
            block_image = Image.open(f"{BUTTON_IMG_PATH}/{block_name}.png")
            block_image = block_image.resize((RESIZE_PX, RESIZE_PX), Image.ANTIALIAS)
            pil_image = block_image.copy()  # Make a copy of the PIL image
            tk_image = ImageTk.PhotoImage(block_image)
            block_button = tk.Button(block_frame, image=tk_image, command=lambda img=tk_image, pil=pil_image: self.select_block(img, pil))
            block_button.image = tk_image  # Keep a reference to the image object
            block_button.pack(padx=5, pady=5)
            self.block_buttons.append(block_button)

        self.canvas = tk.Canvas(self.root, width=300, height=300, bg="white")
        self.canvas.pack(side=tk.LEFT, padx=10, pady=10)
        self.canvas.bind("<Button-1>", self.place_block)
        self.canvas.bind("<Button-3>", self.delete_block)

        export_button = tk.Button(self.root, text="Export Image", command=self.export_image)
        export_button.pack(side=tk.BOTTOM, padx=10, pady=10)

        # Create a dictionary to map canvas items to their respective block images
        self.canvas_items = {}

    def select_block(self, block_image, pil_image):
        self.current_block = block_image
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

    def delete_block(self, event):
        items = self.canvas.find_closest(event.x, event.y)
        if "ghost_block" not in self.canvas.gettags(items):
            self.canvas.delete(items)

    def export_image(self):
        canvas_image = Image.new("RGB", (self.canvas.winfo_width(), self.canvas.winfo_height()), "white")

        # Iterate through canvas items and export them onto the canvas image
        for item, block_image in self.canvas_items.items():
            x0, y0 = self.canvas.coords(item)
            block_image = block_image.pil_image
            block_image = block_image.resize((RESIZE_PX, RESIZE_PX), Image.LANCZOS)  # Resize the image if needed
            canvas_image.paste(block_image, (int(x0), int(y0), int(x0) + RESIZE_PX, int(y0) + RESIZE_PX))

        canvas_image.save(f"{OUTPUT_PATH}/output.png")
        print("Image exported successfully.")

def main():
    root = tk.Tk()
    app = LegoBuilder(root)
    root.mainloop()

if __name__ == "__main__":
    main()
