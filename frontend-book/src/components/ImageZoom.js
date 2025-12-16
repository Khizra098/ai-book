import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import styles from './ImageZoom.module.css';

const ImageZoom = ({ src, alt, caption, ...props }) => {
  const [isZoomed, setIsZoomed] = useState(false);
  const [zoomLevel, setZoomLevel] = useState(1);
  const imgRef = useRef(null);

  const handleZoom = () => {
    setIsZoomed(!isZoomed);
    if (!isZoomed) {
      setZoomLevel(2); // Default zoom level when first opening
    } else {
      setZoomLevel(1); // Reset zoom when closing
    }
  };

  const handleWheel = (e) => {
    if (isZoomed) {
      e.preventDefault();
      e.stopPropagation();
      const delta = e.deltaY > 0 ? -0.2 : 0.2;
      const newZoom = Math.max(1, Math.min(4, zoomLevel + delta));
      setZoomLevel(newZoom);
    }
  };

  const handleOverlayClick = (e) => {
    if (e.target === e.currentTarget) {
      setIsZoomed(false);
      setZoomLevel(1);
    }
  };

  // Handle click outside
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (imgRef.current && !imgRef.current.contains(event.target) && isZoomed) {
        setIsZoomed(false);
        setZoomLevel(1);
      }
    };

    if (isZoomed) {
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isZoomed]);

  return (
    <div className={styles.imageZoomContainer}>
      <div
        ref={imgRef}
        className={clsx(styles.zoomableImage, { [styles.zoomed]: isZoomed })}
        onClick={handleZoom}
        onWheel={handleWheel}
      >
        <img
          src={src}
          alt={alt}
          className={styles.zoomableImg}
          style={{
            cursor: isZoomed ? 'zoom-out' : 'zoom-in',
            transform: `scale(${zoomLevel})`,
          }}
          {...props}
        />
      </div>

      {isZoomed && (
        <div
          className={styles.zoomOverlay}
          onClick={handleOverlayClick}
        >
          <div
            className={styles.zoomedImageContainer}
            onWheel={handleWheel}
          >
            <img
              src={src}
              alt={alt}
              className={styles.zoomedImage}
              style={{ transform: `scale(${zoomLevel})` }}
            />
            <div className={styles.zoomControls}>
              <span>Zoom: {Math.round(zoomLevel * 100)}%</span>
              <button
                className={styles.closeButton}
                onClick={() => {
                  setIsZoomed(false);
                  setZoomLevel(1);
                }}
                type="button"
              >
                Close (ESC)
              </button>
            </div>
          </div>
        </div>
      )}

      {caption && <div className={styles.imageCaption}>{caption}</div>}
    </div>
  );
};

export default ImageZoom;