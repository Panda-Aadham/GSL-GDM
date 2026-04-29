from pathlib import Path
import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


METHODS = ["PMFS", "GrGSL", "Surge-Cast", "Spiral", "Surge-Spiral", "Particle Filter"]

METHOD_FOLDERS = {
    "PMFS": "PMFS",
    "GrGSL": "GrGSL",
    "SurgeCast": "Surge-Cast",
    "Spiral": "Spiral",
    "SurgeSpiral": "Surge-Spiral",
    "ParticleFilter": "Particle Filter",
}

COLORS = {
    "PMFS": "#1f77b4",
    "GrGSL": "#ffb64f",
    "Surge-Cast": "#35a835",
    "Spiral": "#ca3a37",
    "Surge-Spiral": "#9a6dc5",
    "Particle Filter": "#37c2d1",
}

METRICS = {
    "Error": {
        "column": "error",
        "ylabel": "Distance (m)",
        "filename": "avg_error_results.png",
        "value_format": "{:.2f}m",
        "std_format": "± {:.2f}",
    },
    "Time": {
        "column": "search_time",
        "ylabel": "Time (s)",
        "filename": "avg_time_results.png",
        "value_format": "{:.1f}s",
        "std_format": "± {:.1f}",
    },
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot method-level summary charts from synthetic results in results2/."
    )
    parser.add_argument(
        "--results-dir",
        default=".",
        help="Directory containing per-method scenario result CSV files.",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Directory for plots and parsed summary. Defaults to --results-dir.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open the matplotlib windows after saving the plots.",
    )
    return parser.parse_args()


def load_results(results_dir):
    frames = []
    for method_dir in sorted(results_dir.iterdir()):
        if not method_dir.is_dir():
            continue

        method = METHOD_FOLDERS.get(method_dir.name, method_dir.name)
        for result_file in sorted(method_dir.glob("*.csv")):
            if result_file.name == "synthetic_results.csv":
                continue
            if result_file.name.endswith("_variance.csv"):
                continue

            frame = pd.read_csv(result_file)
            frame.insert(0, "method", method)
            frame.insert(1, "source_file", str(result_file))
            frame.insert(2, "scenario_file", result_file.stem)
            frames.append(frame)

    if not frames:
        raise FileNotFoundError(
            f"No per-scenario result CSVs found in {results_dir}. "
            "Expected files like <Method>/House01_1,3-2,4_fast.csv."
        )

    return pd.concat(frames, ignore_index=True)


def successful_results(results):
    if "success" not in results.columns:
        return results
    success = pd.to_numeric(results["success"], errors="coerce")
    return results[success != 0].copy()


def build_summary(results):
    rows = []
    for method in METHODS:
        method_results = results[results["method"] == method]
        if method_results.empty:
            continue
        for metric, config in METRICS.items():
            values = pd.to_numeric(method_results[config["column"]], errors="coerce").dropna()
            rows.append(
                {
                    "Method": method,
                    "Metric": metric,
                    "mean": values.mean(),
                    "std": values.std(ddof=1),
                    "runs": len(values),
                }
            )
    return pd.DataFrame(rows)


def build_file_summary(results):
    rows = []
    grouped = results.groupby(["method", "scenario_file", "source_file"], sort=True)
    for (method, scenario_file, source_file), group in grouped:
        row = {
            "method": method,
            "scenario_file": scenario_file,
            "source_file": source_file,
            "runs": len(group),
        }
        for metric, config in METRICS.items():
            values = pd.to_numeric(group[config["column"]], errors="coerce").dropna()
            key = metric.lower()
            row[f"mean_{key}"] = values.mean()
            row[f"std_{key}"] = values.std(ddof=1)
        rows.append(row)
    return pd.DataFrame(rows)


def plot_metric(summary, metric, output_dir, show=False):
    config = METRICS[metric]
    subset = summary[summary["Metric"] == metric].set_index("Method").reindex(METHODS)

    x = np.arange(len(METHODS))
    means = subset["mean"].to_numpy(dtype=float)
    stds = subset["std"].to_numpy(dtype=float)

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.bar(
        x,
        means,
        color=[COLORS[method] for method in METHODS],
        edgecolor="black",
        linewidth=1.2,
    )

    label_offset = 0.0125 * np.nanmax(means)
    for index, value in enumerate(means):
        if np.isnan(value):
            continue
        ax.text(
            x[index] - 0.04,
            value + label_offset,
            config["value_format"].format(value),
            ha="right",
            va="bottom",
            fontsize=9,
            fontweight="bold",
        )

    for index, (mean, std) in enumerate(zip(means, stds)):
        if np.isnan(mean) or np.isnan(std):
            continue
        ax.text(
            x[index] + 0.04,
            mean + label_offset,
            config["std_format"].format(std),
            ha="left",
            va="bottom",
            fontsize=9,
            fontweight="bold",
        )

    for index, (mean, std) in enumerate(zip(means, stds)):
        if np.isnan(mean) or np.isnan(std):
            continue
        error_bar = ax.errorbar(
            x[index],
            mean,
            yerr=std,
            fmt="none",
            ecolor="black",
            elinewidth=1.5,
            capsize=6,
            capthick=1.5,
        )
        for barline in error_bar[2]:
            barline.set_linestyle("--")

    ax.set_xticks(x)
    ax.set_xticklabels(METHODS, rotation=0, fontweight="bold")
    ax.set_ylabel(config["ylabel"], labelpad=10)
    ax.grid(axis="y", linestyle=":", alpha=0.6)

    plt.tight_layout()
    output_file = output_dir / config["filename"]
    plt.savefig(output_file, dpi=300, bbox_inches="tight")
    if show:
        plt.show()
    else:
        plt.close(fig)

    return output_file


def main():
    args = parse_args()
    results_dir = Path(args.results_dir)
    output_dir = Path(args.output_dir) if args.output_dir else results_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    results = successful_results(load_results(results_dir))
    summary = build_summary(results)
    file_summary = build_file_summary(results)

    summary_file = output_dir / "parsed_summary_results.csv"
    summary.to_csv(summary_file, index=False)
    file_summary_file = output_dir / "parsed_file_summary_results.csv"
    file_summary.to_csv(file_summary_file, index=False)

    print(summary.pivot(index="Method", columns="Metric", values=["mean", "std", "runs"]))
    print(f"\nSaved parsed summary to {summary_file}")
    print(f"Saved per-file summary to {file_summary_file}")

    for metric in METRICS:
        output_file = plot_metric(summary, metric, output_dir, show=args.show)
        print(f"Saved {metric.lower()} plot to {output_file}")


if __name__ == "__main__":
    main()
