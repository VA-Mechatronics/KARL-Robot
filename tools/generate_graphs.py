import ast
import subprocess
from pathlib import Path
from typing import List, Union, Tuple

# --- Configuration ---
PROJECT_ROOT = Path(__file__).resolve().parents[1]
DOCS_DIR = PROJECT_ROOT / "docs" / "graphs"

# Define sources as tuples: (source_path, target_diagram_name)
# source_path can be either a directory or a single file
MODULE_SOURCES: List[Tuple[Path, str]] = [
    (PROJECT_ROOT / "backend" / "controller", "controller"),
    (PROJECT_ROOT / "client", "client"),
    (PROJECT_ROOT / "common" / "kinematics", "kinematics"),
    (PROJECT_ROOT / "common" / "trajectories", "trajectories"),
    (PROJECT_ROOT / "system_assets", "system_assets"),
    (PROJECT_ROOT / "gui" / "desktop", "gui"),
]


class ClassVisitor(ast.NodeVisitor):
    def __init__(self):
        self.classes = []

    def visit_ClassDef(self, node):
        base_names = [base.id for base in node.bases if isinstance(base, ast.Name)]
        methods = []
        attributes = []

        for item in node.body:
            # Methods
            if isinstance(item, ast.FunctionDef):
                if not item.name.startswith("__") or item.name == "__init__":
                    args = [arg.arg for arg in item.args.args if arg.arg != "self"]
                    methods.append(f"+{item.name}({', '.join(args)})")

            # Attributes in __init__
            if isinstance(item, ast.FunctionDef) and item.name == "__init__":
                for stmt in item.body:
                    if isinstance(stmt, ast.Assign):
                        for target in stmt.targets:
                            if isinstance(target, ast.Attribute) and isinstance(target.value, ast.Name) and target.value.id == "self":
                                attributes.append(f"+{target.attr}")

        self.classes.append({
            "name": node.name,
            "bases": base_names,
            "methods": methods,
            "attributes": list(set(attributes))
        })
        self.generic_visit(node)


def parse_source_files(source_path: Path, visitor: ClassVisitor):
    """Recursively parses python files from a directory or a single file."""
    if not source_path.exists():
        print(f"⚠️ Source path does not exist: {source_path}")
        return

    files_to_parse = [source_path] if source_path.is_file() else source_path.glob("**/*.py")

    for py_file in files_to_parse:
        if "venv" in str(py_file) or ".venv" in str(py_file) or py_file.name.startswith("__"):
            continue
        try:
            with open(py_file, "r", encoding="utf-8") as f:
                tree = ast.parse(f.read(), filename=str(py_file))
                visitor.visit(tree)
        except Exception as e:
            print(f"Error reading {py_file}: {e}")


def generate_mermaid(source_path: Path, name: str):
    """Generates Mermaid Markdown (.md) and SVG (.svg) for a given source path."""
    visitor = ClassVisitor()
    parse_source_files(source_path, visitor)

    if not visitor.classes:
        print(f"ℹ️ No classes found for '{name}' in {source_path}")
        return

    title = f"# {name.replace('_', ' ').title()} Architecture\n"

    # Build Mermaid Syntax
    lines = [title, "```mermaid", "classDiagram"]

    for cls in visitor.classes:
        lines.append(f"    class {cls['name']} {{")
        for attr in cls['attributes']:
            lines.append(f"        {attr}")
        for method in cls['methods']:
            lines.append(f"        {method}")
        lines.append("    }")

        for base in cls['bases']:
            lines.append(f"    {base} <|-- {cls['name']}")

    lines.append("```")

    mermaid_code = "\n".join(lines)

    # Ensure output directory exists
    DOCS_DIR.mkdir(parents=True, exist_ok=True)

    md_file = DOCS_DIR / f"{name}.md"
    svg_file = DOCS_DIR / f"{name}.svg"

    md_file.write_text(mermaid_code, encoding="utf-8")
    print(f"✅ MD generated: {md_file}")

    try:
        cmd = f'mmdc -i "{md_file}" -o "{svg_file}" -b transparent'
        print(f"🔄 Executing CLI: {cmd}")

        subprocess.run(cmd, shell=True, check=True)
        print(f"✅ SVG generated: {svg_file}")
    except subprocess.CalledProcessError as e:
        print(f"❌ CLI error (mmdc): {e}")


if __name__ == "__main__":
    print(f"PROJECT_ROOT: {PROJECT_ROOT}")
    print(f"DOCS_DIR: {DOCS_DIR}\n")

    for source_path, name in MODULE_SOURCES:
        generate_mermaid(source_path, name)