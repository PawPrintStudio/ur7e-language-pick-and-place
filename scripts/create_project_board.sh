#!/usr/bin/env bash
# Creates the GitHub Projects v2 board for this repo and adds all open issues.
# Requires: gh CLI authenticated with the `project` scope:
#   gh auth refresh -s project,read:project
set -euo pipefail

ORG="PawPrintStudio"
REPO="${1:?usage: create_project_board.sh <repo-name>}"
TITLE="UR7e Language Pick-and-Place"

echo "Creating project '$TITLE' under org $ORG..."
PROJECT_JSON=$(gh project create --owner "$ORG" --title "$TITLE" --format json)
NUMBER=$(echo "$PROJECT_JSON" | python3 -c 'import json,sys; print(json.load(sys.stdin)["number"])')
echo "Created project #$NUMBER"

echo "Linking repo $ORG/$REPO to project..."
gh project link "$NUMBER" --owner "$ORG" --repo "$ORG/$REPO"

echo "Adding all open issues..."
gh issue list --repo "$ORG/$REPO" --state open --limit 100 --json url --jq '.[].url' |
  while read -r url; do
    gh project item-add "$NUMBER" --owner "$ORG" --url "$url"
    echo "  added $url"
  done

echo "Done: https://github.com/orgs/$ORG/projects/$NUMBER"
