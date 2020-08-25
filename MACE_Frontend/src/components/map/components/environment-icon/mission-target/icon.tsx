import * as React from "react";

type Props = {
  width: number | string;
  height: number | string;
};

export default (props: Props) => (
  <svg
    width={props.width}
    height={props.height}
    viewBox="0 0 512 512"
    fill="none"
    xmlns="http://www.w3.org/2000/svg"
  >
    <path
      d="M256 0C153.755 0 70.573 83.182 70.573 185.426C70.573 312.314 236.512 498.593 243.577 506.461C250.213 513.852 261.799 513.839 268.423 506.461C275.488 498.593 441.427 312.314 441.427 185.426C441.425 83.182 358.244 0 256 0ZM256 278.719C204.558 278.719 162.708 236.868 162.708 185.426C162.708 133.984 204.559 92.134 256 92.134C307.441 92.134 349.291 133.985 349.291 185.427C349.291 236.869 307.441 278.719 256 278.719Z"
      fill="#38A169"
    />
    <circle cx="256" cy="185" r="133" fill="#48BB78" />
    <path
      d="M256 83.5999C200 83.5999 154.6 129 154.6 185C154.6 241 200 286.4 256 286.4C312 286.4 357.4 241 357.4 185C357.4 129 312 83.5999 256 83.5999ZM256 270.8C208.615 270.8 170.2 232.385 170.2 185C170.2 137.615 208.615 99.2002 256 99.2002C303.385 99.2002 341.8 137.615 341.8 185C341.8 232.385 303.385 270.8 256 270.8Z"
      fill="#C6F6D5"
    />
    <path
      d="M369.1 181.1H290.866C289.037 164.814 276.187 151.963 259.9 150.134V71.8998C259.9 69.7472 258.153 68 256 68C253.847 68 252.1 69.7472 252.1 71.8998V150.134C235.814 151.963 222.963 164.813 221.134 181.1H142.9C140.747 181.1 139 182.847 139 185C139 187.153 140.747 188.9 142.9 188.9H221.134C222.963 205.186 235.813 218.037 252.1 219.866V298.1C252.1 300.253 253.847 302 256 302C258.153 302 259.9 300.253 259.9 298.1V219.866C276.186 218.037 289.037 205.187 290.866 188.9H369.1C371.253 188.9 373 187.153 373 185C373 182.847 371.253 181.1 369.1 181.1ZM282.988 181.1H259.9V158.012C271.858 159.751 281.249 169.142 282.988 181.1ZM252.1 158.012V181.1H229.012C230.751 169.142 240.142 159.751 252.1 158.012ZM229.012 188.9H252.1V211.988C240.142 210.249 230.751 200.858 229.012 188.9ZM259.9 211.988V188.9H282.988C281.249 200.858 271.858 210.249 259.9 211.988Z"
      fill="#C6F6D5"
    />
  </svg>
);
