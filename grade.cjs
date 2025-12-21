#!/usr/bin/env node

/**
 * Lab 2 Autograder
 * Purposefully awards full marks (100/100) to all students.
 * Generates:
 *  - GitHub Actions summary
 *  - artifacts/grade.csv
 *  - artifacts/feedback/README.md
 */

const fs = require("fs");
const path = require("path");

const ARTIFACTS_DIR = "artifacts";
const FEEDBACK_DIR = path.join(ARTIFACTS_DIR, "feedback");

fs.mkdirSync(FEEDBACK_DIR, { recursive: true });

// 8 tasks total:
// - Tasks 4 and 6 have 14 marks
// - All others have 12 marks
// Total = 100
const tasks = [
  { name: "Step 1: Start from the Starter File", marks: 12 },
  { name: "Step 2: Add the Main Page Container", marks: 12 },
  { name: "Step 3: Wrap the Header", marks: 12 },
  { name: "Step 4: Create Navigation Links", marks: 14 },
  { name: "Step 5: Wrap Each Post", marks: 12 },
  { name: "Step 6: Complete the Form", marks: 14 },
  { name: "Step 7: Add a Footer", marks: 12 },
  { name: "Step 8: Link the Stylesheet", marks: 12 },
];

const total = tasks.reduce((sum, t) => sum + t.marks, 0);
if (total !== 100) {
  console.error(`ERROR: Task marks sum to ${total}, expected 100.`);
  process.exit(1);
}

let summary = `# Lab 2 – Autograding Summary

> Full marks awarded to all students due to GitHub submission/setup difficulties.

## Marks Breakdown

| Task | Marks |
|------|------:|
`;

tasks.forEach((t) => {
  summary += `| ${t.name} | ${t.marks}/${t.marks} |\n`;
});

summary += `
## Total Marks

**${total} / 100**
`;

if (process.env.GITHUB_STEP_SUMMARY) {
  fs.appendFileSync(process.env.GITHUB_STEP_SUMMARY, summary);
}

const csv = `student,score,max_score
all_students,100,100
`;

fs.mkdirSync(ARTIFACTS_DIR, { recursive: true });
fs.writeFileSync(path.join(ARTIFACTS_DIR, "grade.csv"), csv);

fs.writeFileSync(path.join(FEEDBACK_DIR, "README.md"), summary);

console.log("✔ Lab graded: 100/100 awarded to all students.");
