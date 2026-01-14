/**
 * Content validation test for Chapter 2: Python Agents with rclpy
 * Validates that the content meets educational standards and requirements
 */

const fs = require('fs');
const path = require('path');

// Path to the chapter file
const chapterPath = path.join(__dirname, '../../docs/modules/ros2-humanoid-control/chapter-2-python-agents-rclpy.md');

// Read the chapter content
if (!fs.existsSync(chapterPath)) {
  console.error('❌ Chapter 2 file does not exist:', chapterPath);
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
    name: 'Contains introduction to rclpy section',
    check: () => content.toLowerCase().includes('introduction to rclpy'),
  },
  {
    name: 'Covers Python ROS 2 nodes creation',
    check: () => content.toLowerCase().includes('create') &&
             content.toLowerCase().includes('node') &&
             content.toLowerCase().includes('python'),
  },
  {
    name: 'Explains publishers and subscribers',
    check: () => {
      const lowerContent = content.toLowerCase();
      return lowerContent.includes('publisher') &&
             lowerContent.includes('subscriber');
    },
  },
  {
    name: 'Covers services in Python',
    check: () => content.toLowerCase().includes('service'),
  },
  {
    name: 'Explains bridging AI logic to robot controllers',
    check: () => content.toLowerCase().includes('bridge') &&
             content.toLowerCase().includes('ai') &&
             content.toLowerCase().includes('controller'),
  },
  {
    name: 'Includes Python code examples',
    check: () => content.includes('```python') || content.includes('import rclpy'),
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
    check: () => content.includes('.md') &&
             (content.includes('chapter-1') || content.includes('chapter-3')),
  }
];

// Run validations
let passedCount = 0;
console.log('🧪 Validating Chapter 2: Python Agents with rclpy...\n');

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
  console.log('🎉 Chapter 2 content validation: PASSED');
  process.exit(0);
} else {
  console.log('❌ Chapter 2 content validation: FAILED');
  process.exit(1);
}