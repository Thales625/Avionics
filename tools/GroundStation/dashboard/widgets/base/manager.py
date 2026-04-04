from .widget import Widget

class WidgetManager:
    def __init__(self):
        self.widgets = []

    def register(self, widget: Widget):
        self.widgets.append(widget)

    def tick(self, now):
        for widget in self.widgets:
            if not widget.active: continue
            if widget.interval is not None and now - widget._last_update < widget.interval: continue
            if not widget.persistent and (not widget.isVisible() or widget.visibleRegion().isEmpty()): continue

            widget._last_update = now

            widget.tick()
