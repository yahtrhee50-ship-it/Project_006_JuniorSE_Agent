# Projects

Each subfolder is one engineering project. Copy `_TEMPLATE` to start a new project.

## To start a new project

1. Copy `_TEMPLATE` folder, rename it to your project name
2. Fill in `project.json` with project-specific values
3. Drop Excel load schedules into `input\`
4. Tell the agent: "Load project [ProjectName]" to begin a session

## Folder contents

| Folder/File | Purpose |
|---|---|
| `project.json` | Always-loaded context (risk category, codes, materials, key loads) |
| `input\` | User input files: Excel load schedules, plain text task descriptions |
| `members\inventory.json` | Running inventory of members sized or checked on this project |
| `calcs\` | Dated calc output files — one per task, never overwritten (audit trail) |
| `sessions\` | Compact session summaries — on-demand memory tier |
