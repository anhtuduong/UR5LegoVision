import tkinter as tk
from PIL import Image, ImageTk, ImageDraw

class LegoConstructionTool:
    def __init__(self, root):
        self.root = root
        self.root.title("Lego Construction Tool")

        self.current_block = None
        self.ghost_block = None

        self.create_widgets()

    def create_widgets(self):
        block_frame = tk.Frame(self.root)
        block_frame.pack(side=tk.LEFT, padx=10, pady=10)

        block_names = ["Block_A"]
        self.block_buttons = []
        for block_name in block_names:
            block_image = Image.open(f"{block_name}.png")
            block_image = block_image.resize((30, 30), Image.ANTIALIAS)
            block_image = ImageTk.PhotoImage(block_image)
            block_button = tk.Button(block_frame, image=block_image, command=lambda img=block_image: self.select_block(img))
            block_button.image = block_image  # Keep a reference to the image object
            block_button.pack(padx=5, pady=5)
            self.block_buttons.append(block_button)

        self.canvas = tk.Canvas(self.root, width=400, height=400, bg="white")
        self.canvas.pack(side=tk.LEFT, padx=10, pady=10)
        self.canvas.bind("<Button-1>", self.place_block)
        self.canvas.bind("<Button-3>", self.delete_block)

        export_button = tk.Button(self.root, text="Export Image", command=self.export_image)
        export_button.pack(side=tk.BOTTOM, padx=10, pady=10)

    def select_block(self, block_image):
        self.current_block = block_image
        self.canvas.delete("ghost_block")
        self.ghost_block = self.canvas.create_image(10, 10, anchor=tk.NW, image=self.current_block, tags="ghost_block")

    def place_block(self, event):
        if self.current_block is not None:
            self.canvas.create_image(event.x, event.y, anchor=tk.NW, image=self.current_block)

    def delete_block(self, event):
        items = self.canvas.find_closest(event.x, event.y)
        if "ghost_block" not in self.canvas.gettags(items):
            self.canvas.delete(items)

    def export_image(self):
        canvas_image = Image.new("RGB", (self.canvas.winfo_width(), self.canvas.winfo_height()), "white")
        for item in self.canvas.find_all():
            if self.canvas.type(item) == 'image':
                x0, y0 = self.canvas.coords(item)
                block_image = self.canvas.itemcget(item, "image")
                block_image = Image.open(block_image)
                block_image = block_image.resize((30, 30), Image.LANCZOS)  # Resize the image if needed
                canvas_image.paste(block_image, (int(x0), int(y0), int(x0) + 30, int(y0) + 30))

        canvas_image.save("output.png")
        print("Image exported successfully.")

def main():
    root = tk.Tk()
    app = LegoConstructionTool(root)
    root.mainloop()

if __name__ == "__main__":
    main()
