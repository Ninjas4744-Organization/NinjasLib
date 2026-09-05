# Builds the guide site (_src/) and publishes it into this folder (docs/), next to the
# checked-in javadoc at docs/javadoc/. Run this after editing any file under docs/_src/, and
# commit the result - docs/ is what GitHub Pages serves, and it's the only folder anyone
# downloading the library needs to deal with; every doc-related file (source, tooling, venv,
# javadoc, and the published site itself) lives under here.
#
# One-time setup (only needed once, or after requirements-docs.txt changes), run from this
# folder (docs/):
#   python -m venv .venv-docs
#   .venv-docs\Scripts\pip.exe install -r requirements-docs.txt
#
# To update the javadoc itself: regenerate it with your usual javadoc process and copy the
# output directly into docs/javadoc/ (it's never touched by this script).

$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$tempBuild = "$root\.mkdocs-build"

& "$root\.venv-docs\Scripts\python.exe" -m mkdocs build -f "$root\mkdocs.yml"

# mkdocs build writes into $tempBuild (see site_dir in mkdocs.yml) instead of straight into this
# folder, because docs/_src (the markdown source), docs/javadoc (the checked-in javadoc), and
# this script's own tooling files all live here too and must survive every rebuild. Mirror the
# fresh build into this folder instead, excluding everything that isn't part of the built site.
robocopy $tempBuild $root /MIR `
    /XD "$root\_src" "$root\javadoc" "$root\.venv-docs" "$root\.mkdocs-build" `
    /XF "$root\mkdocs.yml" "$root\build-docs.ps1" "$root\requirements-docs.txt" "$root\.gitignore" `
    /NFL /NDL /NJH /NJS
if ($LASTEXITCODE -ge 8) {
    throw "robocopy failed while publishing the build into docs/ (exit code $LASTEXITCODE)"
}

Remove-Item -Path $tempBuild -Recurse -Force

# The mirror step above can delete docs/.nojekyll (it's not part of the mkdocs build output),
# so recreate it every time: it tells GitHub Pages to serve docs/ as plain static files instead
# of running it through Jekyll.
New-Item -ItemType File -Path "$root\.nojekyll" -Force | Out-Null

Write-Host "Site built into docs/. Review the changes and commit them."
