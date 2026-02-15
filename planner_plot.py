import numpy as np
import plotly.graph_objects as go
import plotly.io as pio


def plot_trapezoid(time_s, position_mm, boundaries_s=None):
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

    if len(time_s) < 3:
        velocity_mms = np.array([np.nan] * len(time_s))
        acceleration_mmss = np.array([np.nan] * len(time_s))
        jerk_mmsss = np.array([np.nan] * len(time_s))
    else:
        velocity_mms = np.gradient(position_mm, time_s, edge_order=2)
        acceleration_mmss = np.gradient(velocity_mms, time_s, edge_order=2)
        jerk_mmsss = np.gradient(acceleration_mmss, time_s, edge_order=2)

    smooth_window = 1

    if len(time_s) >= smooth_window:
        kernel = np.ones(smooth_window) / smooth_window
        velocity_mms = np.convolve(velocity_mms, kernel, mode="same")
        acceleration_mmss = np.convolve(acceleration_mmss, kernel, mode="same")
        jerk_mmsss = np.convolve(jerk_mmsss, kernel, mode="same")

    edge_trim = max(3, smooth_window // 2)
    if len(time_s) >= edge_trim * 2:
        velocity_mms[:edge_trim] = np.nan
        velocity_mms[-edge_trim:] = np.nan
        acceleration_mmss[:edge_trim] = np.nan
        acceleration_mmss[-edge_trim:] = np.nan
        jerk_mmsss[:edge_trim] = np.nan
        jerk_mmsss[-edge_trim:] = np.nan

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

    traces.append(
        go.Scattergl(
            x=time_s,
            y=acceleration_mmss,
            mode="lines",
            line=dict(width=2, color="lime"),
            name="acceleration",
            xaxis="x3",
            yaxis="y3",
        )
    )

    traces.append(
        go.Scattergl(
            x=time_s,
            y=jerk_mmsss,
            mode="lines",
            line=dict(width=2, color="violet"),
            name="jerk",
            xaxis="x4",
            yaxis="y4",
        )
    )

    layout = dict(
        grid=dict(rows=4, columns=1, pattern="independent"),
        xaxis=dict(showticklabels=True),
        xaxis2=dict(matches="x", showticklabels=True, title="time (s)"),
        yaxis=dict(title="position (mm)"),
        yaxis2=dict(title="velocity (mm/s)"),
        height=1400,
        showlegend=True,
    )

    layout["xaxis3"] = dict(matches="x", showticklabels=True, title="time (s)")
    layout["yaxis3"] = dict(title="acceleration (mm/s^2)")
    layout["xaxis4"] = dict(matches="x", showticklabels=True, title="time (s)")
    layout["yaxis4"] = dict(title="jerk (mm/s^3)")

    fig = go.Figure(data=traces, layout=layout)

    if boundaries_s is not None:
        for bt in boundaries_s:
            fig.add_vline(x=bt, line_width=1, line_color="#888", opacity=0.4)

    fig.show(config={"displaylogo": False, "responsive": True})
