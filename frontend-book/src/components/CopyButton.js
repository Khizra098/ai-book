import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './CopyButton.module.css';

const CopyButton = ({ code, onCopy, copied: externalCopied }) => {
  const [internalCopied, setInternalCopied] = useState(false);
  const isCopied = externalCopied || internalCopied;

  const handleCopy = async () => {
    try {
      await navigator.clipboard.writeText(code);
      setInternalCopied(true);
      onCopy && onCopy();
      setTimeout(() => setInternalCopied(false), 2000);
    } catch (err) {
      console.error('Failed to copy text: ', err);
    }
  };

  return (
    <button
      className={clsx('button button--secondary button--sm', styles.copyButton, {
        [styles.copied]: isCopied,
      })}
      onClick={handleCopy}
      title="Copy to clipboard"
      type="button"
    >
      <span className={styles.buttonContent}>
        {isCopied ? (
          <>
            <span className={styles.checkmark}>✓</span> Copied!
          </>
        ) : (
          <>
            <span className={styles.copyIcon}>📋</span> Copy
          </>
        )}
      </span>
    </button>
  );
};

export default CopyButton;