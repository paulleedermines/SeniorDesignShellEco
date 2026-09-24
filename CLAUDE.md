@AGENTS.md

# Claude Code notes

- **Branch:** commit and push only to `claude` (`git push` is already set to track `origin/claude`). Never push to `main`; changes reach `main` through a pull request.
- **Running MATLAB:** use the PowerShell tool with `& "C:\Program Files\MATLAB\R2026a\bin\matlab.exe" -batch "..."` and a long timeout (600000 ms). Start-up alone takes about 20 s, so combine checks into one call. Always hide figures with `set(groot,'DefaultFigureVisible','off')`.
- **Scratch files:** analysis scripts, CSV dumps and throwaway MATLAB drivers go in the session scratchpad, not the repo.
- **`Old Stuff/`:** don't edit it unless the user asks. New code goes in `lapsim/` (see AGENTS.md).
- **Quoting numbers:** before stating a lap time, energy or km/kWh figure, run the model and quote the real output. If a result disagrees with the benchmark in README §2, say so and find out why rather than adjusting it away.
- **Keeping docs current:** after fixing a legacy issue in the new model, or sourcing a parameter, update README §7 / §7.2 and AGENTS.md in the same commit.
