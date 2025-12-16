import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './EnhancedCodeBlock.module.css';

const EnhancedCodeBlock = ({ children, className = '', title, showLineNumbers = false }) => {
  const [copied, setCopied] = useState(false);

  const handleCopy = async () => {
    try {
      await navigator.clipboard.writeText(children.trim());
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    } catch (err) {
      console.error('Failed to copy text: ', err);
    }
  };

  return (
    <div className={clsx('code-block-container', styles.codeBlockContainer, className)}>
      {title && <div className={styles.codeBlockTitle}>{title}</div>}
      <div className={styles.codeBlockHeader}>
        <div className={styles.codeBlockActions}>
          <button
            className={clsx('button button--secondary button--sm', styles.copyButton)}
            onClick={handleCopy}
            title="Copy code to clipboard"
            type="button"
          >
            {copied ? '✓ Copied!' : 'Copy'}
          </button>
        </div>
      </div>
      <pre className={clsx('code-block-pre', styles.codeBlockPre)}>
        <code className={className.replace('code-block-container ', '')}>
          {children}
        </code>
      </pre>
    </div>
  );
};

export default EnhancedCodeBlock;