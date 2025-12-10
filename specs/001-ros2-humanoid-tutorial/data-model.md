# Data Model: Docusaurus Book for Physical AI & Humanoid Robotics

## Overview
This document describes the data models and content structure for the Docusaurus-based book on Physical AI & Humanoid Robotics.

## Content Entities

### Chapter
- **name**: String - The title of the chapter
- **slug**: String - URL-friendly identifier for the chapter
- **order**: Integer - Sequential ordering of chapters (1-6)
- **sections**: Array of Section objects - The lessons within the chapter
- **description**: String - Brief summary of the chapter content
- **learning_objectives**: Array of strings - What users will learn
- **prerequisites**: Array of strings - Knowledge required before reading

### Section (Lesson)
- **title**: String - The title of the section
- **filename**: String - The markdown filename (e.g., "01-overview.md")
- **order**: Integer - Sequential ordering within the chapter
- **content_type**: Enum (text, code_example, diagram, exercise) - Type of content
- **word_count**: Integer - Estimated word count for the section
- **estimated_reading_time**: Integer - Time in minutes to read the section
- **dependencies**: Array of strings - Other sections this section depends on

### Content Block
- **type**: Enum (paragraph, code_block, image, diagram, exercise, reference) - Type of content block
- **content**: String - The actual content
- **metadata**: Object - Additional information based on type
  - For code_block: {language, caption, filename}
  - For image: {src, alt, caption}
  - For exercise: {difficulty, solution_available}
  - For reference: {citation_type, source}

### Reference/Citation
- **id**: String - Unique identifier for the citation
- **type**: Enum (book, article, website, documentation, paper) - Type of source
- **title**: String - Title of the source
- **authors**: Array of strings - Authors of the source
- **publisher**: String - Publisher or source location
- **publication_date**: String - Date in YYYY-MM-DD format
- **url**: String - URL if applicable
- **accessed_date**: String - Date accessed in YYYY-MM-DD format
- **apa_citation**: String - Full APA format citation

### Media Asset
- **filename**: String - Name of the file
- **path**: String - Relative path from static directory
- **type**: Enum (image, diagram, video, audio) - Type of media
- **alt_text**: String - Accessibility text
- **caption**: String - Descriptive caption
- **usage**: Array of strings - Which chapters/sections use this asset
- **size**: Integer - File size in bytes
- **dimensions**: Object - Width and height for images {width, height}

### Code Example
- **id**: String - Unique identifier
- **title**: String - Brief description
- **language**: String - Programming language (python, xml for URDF, etc.)
- **filename**: String - Name of the file in examples directory
- **content**: String - The actual code
- **description**: String - What the code demonstrates
- **usage_context**: String - Where this code is referenced in the book
- **dependencies**: Array of strings - ROS packages or libraries needed

## Content Relationships

### Chapter to Section
- One-to-Many: Each chapter contains multiple sections
- Sections are ordered within the chapter
- Sections belong to exactly one chapter

### Section to Content Block
- One-to-Many: Each section contains multiple content blocks
- Content blocks are ordered within the section
- Content blocks belong to exactly one section

### Section to Reference
- Many-to-Many: Sections can reference multiple sources, and sources can be referenced by multiple sections
- Each reference in a section has a specific citation location

### Section to Code Example
- Many-to-Many: Sections can reference multiple code examples, and code examples can be referenced by multiple sections
- Each reference has a specific context

### Section to Media Asset
- Many-to-Many: Sections can use multiple media assets, and media assets can be used by multiple sections
- Each usage has a specific context and placement

## Validation Rules from Requirements

1. **Word Count Constraint**: Each section must contribute to the total word count of 3000-5000 words across the entire book
2. **APA Citation Compliance**: All references must follow APA format as required
3. **Content Accuracy**: All technical content must be verified against official ROS 2, Isaac Sim, and Gazebo documentation
4. **Chapter Structure**: Must follow the 6-chapter structure as specified
5. **Exercise Inclusion**: Chapter 6 must include exercises as specified
6. **Reference Inclusion**: Chapter 6 must include references as specified
7. **Simulation Focus**: Content must focus on ROS 2 simulation (not hardware-specific deployment) as specified
8. **Scope Boundaries**: Must NOT include full humanoid AI integration or voice-to-action concepts as specified

## State Transitions (if applicable)

### Content Status
- **draft**: Initial state when content is being written
- **review**: Content is ready for technical review
- **verified**: Content has been verified against official documentation
- **published**: Content is ready for inclusion in the final book
- **deprecated**: Content is no longer relevant (rare for this project type)