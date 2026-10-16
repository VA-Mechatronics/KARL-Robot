import sys
from pathlib import Path
import pandas as pd
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import webbrowser

# Locate telemetry logs
PROJECT_ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = PROJECT_ROOT / "workspace" / "telemetry_logs"

# Configurable time shift to compensate latency
TIME_SHIFT = 0.20  # Set e.g. to 0.20 for 200ms shift


def get_latest_log_file() -> Path | None:
    files = sorted(LOG_DIR.glob("telemetry_*.parquet"))
    return files[-1] if files else None


def plot_telemetry(file_path: Path):
    print(f"Loading telemetry file: {file_path}")
    df = pd.read_parquet(file_path)

    # Calculate average sampling dt in seconds
    dt = df["rel_time"].diff().median() if len(df) > 1 else 0.01
    shift_samples = int(round(TIME_SHIFT / dt)) if dt > 0 else 0

    # Calculate additional metrics dynamically in Pandas
    for i in range(7):
        # Shift set_pos backward/forward to align with act_pos timeline
        if shift_samples != 0:
            df[f"j{i + 1}_set_pos_aligned"] = df[f"j{i + 1}_set_pos"].shift(shift_samples)
        else:
            df[f"j{i + 1}_set_pos_aligned"] = df[f"j{i + 1}_set_pos"]

        df[f"j{i + 1}_pos_diff"] = df[f"j{i + 1}_set_pos_aligned"] - df[f"j{i + 1}_act_pos"]

    colors = ['#E41A1C', '#4DAF4A', '#377EB8', '#984EA3', '#FF7F00', '#A65628', '#F781BF']

    fig = make_subplots(
        rows=5, cols=1,
        shared_xaxes=True,
        vertical_spacing=0.04,
    )

    # 1. Subplot: Joint Positions (Actual & Set)
    for i in range(7):
        # Actual Position (Solid)
        fig.add_trace(go.Scatter(
            x=df["rel_time"], y=df[f"j{i+1}_act_pos"],
            mode='lines', name=f"J{i+1} Act",
            line=dict(color=colors[i], width=2),
            legendgroup=f"j{i+1}",
            hovertemplate=f"J{i+1} Act: %{{y:.2f}}°<extra></extra>"
        ), row=1, col=1)

        # Set Position (Dashed)
        fig.add_trace(go.Scatter(
            x=df["rel_time"], y=df[f"j{i + 1}_set_pos_aligned"],
            mode='lines', name=f"J{i+1} Set",
            line=dict(color=colors[i], width=1.5, dash='dash'),
            legendgroup=f"j{i+1}",
            hovertemplate=f"J{i+1} Set: %{{y:.2f}}°<extra></extra>"
        ), row=1, col=1)

    # 2. Subplot: Calculated Position Delta (Set - Act)
    for i in range(7):
        max_diff = df[f"j{i + 1}_pos_diff"].abs().max()
        fig.add_trace(go.Scatter(
            x=df["rel_time"], y=df[f"j{i + 1}_pos_diff"],
            mode='lines', name=f"J{i + 1} Delta",
            line=dict(color=colors[i], width=1.5),
            legendgroup=f"j{i + 1}", showlegend=False,
            hovertemplate=f"J{i + 1} ΔPos: %{{y:.3f}}° (Max: {max_diff:.3f}°)<extra></extra>"
        ), row=2, col=1)

    # 3. Subplot: Following Error (err)
    for i in range(7):
        fig.add_trace(go.Scatter(
            x=df["rel_time"], y=df[f"j{i+1}_err"],
            mode='lines', name=f"J{i+1} Err",
            line=dict(color=colors[i], width=1.5),
            legendgroup=f"j{i+1}", showlegend=False,
            hovertemplate=f"J{i+1} Err: %{{y:.3f}}°<extra></extra>"
        ), row=3, col=1)

    # 4. Subplot: Velocity (act_vel)
    for i in range(7):
        fig.add_trace(go.Scatter(
            x=df["rel_time"], y=df[f"j{i+1}_act_vel"],
            mode='lines', name=f"J{i+1} Vel",
            line=dict(color=colors[i], width=1.5),
            legendgroup=f"j{i+1}", showlegend=False,
            hovertemplate=f"J{i+1} Vel: %{{y:.2f}}°/s<extra></extra>"
        ), row=4, col=1)

    # 5. Subplot: Motor Current
    for i in range(7):
        fig.add_trace(go.Scatter(
            x=df["rel_time"], y=df[f"j{i+1}_current"],
            mode='lines', name=f"J{i+1} Current",
            line=dict(color=colors[i], width=1.5),
            legendgroup=f"j{i+1}", showlegend=False,
            hovertemplate=f"J{i+1} Curr: %{{y:.0f}}mA<extra></extra>"
        ), row=5, col=1)

    title_suffix = f" (Shift: {TIME_SHIFT*1000:.0f}ms)" if TIME_SHIFT != 0 else ""

    # Layout
    fig.update_layout(
        title=dict(text=f"Telemetry Log Trace: {file_path.name}{title_suffix}", font=dict(size=18), x=0.0, xanchor="left"),
        height=1500,
        template=None,
        hovermode="x",
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="right",
            x=1,
        )
    )
    fig.update_yaxes(showgrid=True)

    # Set Axis Labels
    fig.update_xaxes(title_text="Time [s]", row=5, col=1)
    fig.update_yaxes(title_text="Pos [deg]", row=1, col=1)
    fig.update_yaxes(title_text="Pos Diff [deg]", row=2, col=1)
    fig.update_yaxes(title_text="Err [deg]", row=3, col=1)
    fig.update_yaxes(title_text="Vel [deg/s]", row=4, col=1)
    fig.update_yaxes(title_text="Current [mA]", row=5, col=1)

    # Save to HTML and auto-open in default browser
    output_html = file_path.with_suffix(".html")
    fig.write_html(output_html)
    print(f"Opening interactive plot in browser: {output_html}")
    webbrowser.open(output_html.as_uri())


if __name__ == "__main__":
    target_file = None
    if len(sys.argv) > 1:
        target_file = Path(sys.argv[1])
    else:
        target_file = get_latest_log_file()

    if target_file and target_file.exists():
        plot_telemetry(target_file)
    else:
        print("No telemetry parquet files found in workspace/telemetry_logs/")