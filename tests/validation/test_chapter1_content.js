/**
 * Content validation test for Chapter 1: ROS 2 Fundamentals
 * Validates that the content meets educational standards and requirements
 */

const fs = require('fs');
const path = require('path');

// Path to the chapter file
const chapterPath = path.join(__dirname, '../../docs/modules/ros2-humanoid-control/chapter-1-ros2-fundamentals.md');

// Read the chapter content
if (!fs.existsSync(chapterPath)) {
  console.error('❌ Chapter 1 file does not exist:', chapterPath);
  process.exit(1);
}

const content = fs.readFileSync(chapterPath, 'utf8');

// Define validation criteria
const validations = [
  {
    name: 'Has proper frontmatter metadata',
    check: () => content.startsWith('---'),
  },
  {
    name: 'Contains introduction to ROS 2 section',
    check: () => content.toLowerCase().includes('introduction to ros 2'),
  },
  {
    name: 'Explains core concepts: nodes, topics, services',
    check: () => {
      const lowerContent = content.toLowerCase();
      return lowerContent.includes('nodes') &&
             lowerContent.includes('topics') &&
             lowerContent.includes('services');
    },
  },
  {
    name: 'Covers role of ROS 2 in physical AI',
    check: () => content.toLowerCase().includes('role of ros 2 in physical ai'),
  },
  {
    name: 'Explains message-based robot control',
    check: () => content.toLowerCase().includes('message-based robot control'),
  },
  {
    name: 'Has summary or learning objectives section',
    check: () => content.toLowerCase().includes('summary') ||
             content.toLowerCase().includes('learning objectives'),
  },
  {
    name: 'Has sufficient content length (> 1000 characters)',
    check: () => content.length > 1000,
  },
  {
    name: 'Includes navigation links',
    check: () => content.includes('.md') && content.includes('chapter-2'),
  }
];

// Run validations
let passedCount = 0;
console.log('🧪 Validating Chapter 1: ROS 2 Fundamentals...\n');

validations.forEach((validation, index) => {
  const passed = validation.check();
  const status = passed ? '✅' : '❌';
  console.log(`${status} ${index + 1}. ${validation.name}`);
  if (!passed) {
    console.log(`   Failed: Content does not meet "${validation.name}" requirement`);
  } else {
    passedCount++;
  }
});

console.log(`\n📊 Results: ${passedCount}/${validations.length} validations passed`);

if (passedCount === validations.length) {
  console.log('🎉 Chapter 1 content validation: PASSED');
  process.exit(0);
} else {
  console.log('❌ Chapter 1 content validation: FAILED');
  process.exit(1);
}