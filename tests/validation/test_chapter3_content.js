/**
 * Content validation test for Chapter 3: Humanoid Modeling with URDF
 * Validates that the content meets educational standards and requirements
 */

const fs = require('fs');
const path = require('path');

// Path to the chapter file
const chapterPath = path.join(__dirname, '../../docs/modules/ros2-humanoid-control/chapter-3-urdf-modeling.md');

// Read the chapter content
if (!fs.existsSync(chapterPath)) {
  console.error('❌ Chapter 3 file does not exist:', chapterPath);
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
    name: 'Contains introduction to URDF section',
    check: () => content.toLowerCase().includes('introduction to urdf'),
  },
  {
    name: 'Explains the purpose of URDF',
    check: () => content.toLowerCase().includes('purpose of urdf'),
  },
  {
    name: 'Covers links, joints, and kinematic chains',
    check: () => {
      const lowerContent = content.toLowerCase();
      return lowerContent.includes('links') &&
             lowerContent.includes('joints') &&
             lowerContent.includes('kinematic chains');
    },
  },
  {
    name: 'Explains URDF\'s role in ROS 2 and simulation',
    check: () => content.toLowerCase().includes('urdf') &&
             content.toLowerCase().includes('ros 2') &&
             content.toLowerCase().includes('simulation'),
  },
  {
    name: 'Includes URDF examples (XML snippets)',
    check: () => content.includes('<link') || content.includes('<joint'),
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
console.log('🧪 Validating Chapter 3: Humanoid Modeling with URDF...\n');

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
  console.log('🎉 Chapter 3 content validation: PASSED');
  process.exit(0);
} else {
  console.log('❌ Chapter 3 content validation: FAILED');
  process.exit(1);
}