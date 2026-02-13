import plotly.graph_objects as go
import plotly.io as pio


def plot_trapezoid(time_s, position_mm, velocity_mms, boundaries_s=None):
    pio.templates.default = pio.templates["plotly_dark"]
    pio.templates.default.layout.height = 400
    pio.templates.default.layout.xaxis.gridcolor = "#999"
    pio.templates.default.layout.yaxis.gridcolor = "#999"

    traces = []

    traces.append(
        go.Scattergl(
            x=time_s,
            y=position_mm,
            mode="lines",
            line=dict(width=2, color="deepskyblue"),
            name="position",
            xaxis="x",
            yaxis="y",
        )
    )

    traces.append(
        go.Scattergl(
            x=time_s,
            y=velocity_mms,
            mode="lines",
            line=dict(width=2, color="orange"),
            name="velocity",
            xaxis="x2",
            yaxis="y2",
        )
    )

    layout = dict(
        grid=dict(rows=2, columns=1, pattern="independent"),
        xaxis=dict(showticklabels=True),
        xaxis2=dict(matches="x", showticklabels=True, title="time (s)"),
        yaxis=dict(title="position (mm)"),
        yaxis2=dict(title="velocity (mm/s)"),
        height=900,
        showlegend=True,
    )

    fig = go.Figure(data=traces, layout=layout)

    if boundaries_s is not None:
        for bt in boundaries_s:
            fig.add_vline(x=bt, line_width=1, line_color="#888", opacity=0.4)

    fig.show(config={"displaylogo": False, "responsive": True})
