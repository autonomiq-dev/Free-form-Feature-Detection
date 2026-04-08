import os
import sys
import subprocess
from pathlib import Path



def step_to_stl(
    step_path: str,
    output_dir: str | None = None,
    linear_deflection: float = 0.1,
    angular_deflection: float = 0.5,
) -> str:
    
    step_path = _resolve_input(step_path)
    stl_path = _build_output_path(step_path, output_dir)

    print(f"[step_to_stl] Input : {step_path}")
    print(f"[step_to_stl] Output: {stl_path}")

    # Try each backend in priority order
    for backend_fn in (_convert_pythonocc, _convert_cadquery, _convert_freecad):
        try:
            backend_fn(step_path, stl_path, linear_deflection, angular_deflection)
            print(f"[step_to_stl] ✓ Done – {stl_path}")
            return str(stl_path)
        except ImportError:
            continue            # backend not installed, try next
        except Exception as exc:
            raise RuntimeError(f"Conversion failed: {exc}") from exc

    raise RuntimeError(
        "No CAD backend found. Install one of:\n"
        "  conda install -c conda-forge pythonocc-core\n"
        "  pip install cadquery\n"
        "  apt install freecad  (Linux)"
    )


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _resolve_input(path: str) -> Path:
    """Handle local paths and optional HTTP URLs."""
    if path.startswith("http://") or path.startswith("https://"):
        return _download_step(path)

    p = Path(path).expanduser().resolve()
    if not p.exists():
        raise FileNotFoundError(f"STEP file not found: {p}")
    return p


def _download_step(url: str) -> Path:
    """Download a remote STEP file to a temp location."""
    try:
        import requests
        import tempfile
    except ImportError:
        raise ImportError("Install 'requests' to use URL inputs: pip install requests")

    filename = url.split("/")[-1].split("?")[0] or "downloaded.step"
    tmp_dir = Path(tempfile.mkdtemp())
    dest = tmp_dir / filename
    print(f"[step_to_stl] Downloading {url} …")
    resp = requests.get(url, timeout=60)
    resp.raise_for_status()
    dest.write_bytes(resp.content)
    return dest


def _build_output_path(step_path: Path, output_dir: str | None) -> Path:
    """Build the output .stl path, keeping the same stem as the input."""
    stem = step_path.stem                          # e.g. "ff_case_1"
    out_dir = Path(output_dir) if output_dir else step_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir / f"{stem}.stl"


# ---------------------------------------------------------------------------
# Backend 1 – pythonOCC  (pip install pythonocc-core)
# ---------------------------------------------------------------------------

def _convert_pythonocc(
    step_path: Path,
    stl_path: Path,
    linear_deflection: float,
    angular_deflection: float,
) -> None:
    """Convert using pythonOCC (OpenCASCADE Python bindings)."""
    from OCC.Core.STEPControl import STEPControl_Reader
    from OCC.Core.IFSelect import IFSelect_RetDone
    from OCC.Core.BRep import BRep_Builder
    from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
    from OCC.Core.StlAPI import StlAPI_Writer
    from OCC.Core.TopoDS import TopoDS_Compound
    from OCC.Core.BRep import BRep_Builder

    print("[step_to_stl] Backend: pythonOCC")

    # Read STEP
    reader = STEPControl_Reader()
    status = reader.ReadFile(str(step_path))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"STEPControl_Reader failed (status {status})")

    reader.TransferRoots()
    shape = reader.OneShape()

    # Mesh the shape
    mesh = BRepMesh_IncrementalMesh(shape, linear_deflection, False, angular_deflection)
    mesh.Perform()
    if not mesh.IsDone():
        raise RuntimeError("Meshing failed")

    # Write STL
    writer = StlAPI_Writer()
    writer.SetASCIIMode(False)          # binary STL (smaller file)
    result = writer.Write(shape, str(stl_path))
    if not result:
        raise RuntimeError("StlAPI_Writer failed to write the file")


# ---------------------------------------------------------------------------
# Backend 2 – cadquery  (pip install cadquery)
# ---------------------------------------------------------------------------

def _convert_cadquery(
    step_path: Path,
    stl_path: Path,
    linear_deflection: float,
    angular_deflection: float,
) -> None:
    """Convert using CadQuery (also built on OpenCASCADE)."""
    import cadquery as cq

    print("[step_to_stl] Backend: cadquery")
    shape = cq.importers.importStep(str(step_path))
    cq.exporters.export(
        shape,
        str(stl_path),
        exportType=cq.exporters.ExportTypes.STL,
        tolerance=linear_deflection,
        angularTolerance=angular_deflection,
    )


# ---------------------------------------------------------------------------
# Backend 3 – FreeCAD CLI  (system install)
# ---------------------------------------------------------------------------

_FREECAD_SCRIPT = """\
import sys, FreeCAD, Part, Mesh
step_path, stl_path = sys.argv[1], sys.argv[2]
Part.open(step_path)
doc = FreeCAD.ActiveDocument
objs = doc.Objects
shape = Part.makeCompound([o.Shape for o in objs if hasattr(o, 'Shape')])
mesh = doc.addObject("Mesh::Feature", "Mesh")
mesh.Mesh = Mesh.MeshObject()
Mesh.export([mesh], stl_path)
print("FreeCAD export done")
"""

def _convert_freecad(
    step_path: Path,
    stl_path: Path,
    linear_deflection: float,
    angular_deflection: float,
) -> None:
    """Convert via FreeCAD command-line (requires system FreeCAD install)."""
    import tempfile, shutil

    freecad_bin = shutil.which("FreeCADCmd") or shutil.which("freecadcmd")
    if not freecad_bin:
        raise ImportError("FreeCAD CLI not found in PATH")

    print("[step_to_stl] Backend: FreeCAD CLI")

    # Write the helper script to a temp file
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".py", delete=False
    ) as tmp:
        tmp.write(_FREECAD_SCRIPT)
        script_path = tmp.name

    try:
        result = subprocess.run(
            [freecad_bin, script_path, str(step_path), str(stl_path)],
            capture_output=True,
            text=True,
            timeout=120,
        )
        if result.returncode != 0:
            raise RuntimeError(result.stderr or result.stdout)
    finally:
        os.unlink(script_path)


# ---------------------------------------------------------------------------
# CLI usage:  python step_to_stl.py my_file.STEP [output_dir]
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python step_to_stl.py <file.STEP> [output_dir]")
        sys.exit(1)

    input_file = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else None

    try:
        result_path = step_to_stl(input_file, output_dir=out_dir)
        print(f"STL saved to: {result_path}")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)