from .widget import Widget

class WidgetManager:
    def __init__(self):
        self.widgets = []

    def register(self, widget: Widget):
        self.widgets.append(widget)

    def tick(self):
        for widget in self.widgets:
            if not widget.active: continue
            if not widget.persistent and not widget.isVisible(): continue

            widget.tick()
